"""Read-only verification of design assets and pinned upstream sources.

No network, package installation, camera, SSH, or CAN access.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(data)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache-root', type=Path)
    parser.add_argument('--check-baseline', action='store_true',
                        help='Also check that reviewed v5/common files have not changed.')
    args = parser.parse_args()
    lock = json.loads((ROOT/'dependencies/sources.lock.json').read_text())
    cache = args.cache_root or Path(lock['source_cache_root'])
    failures = []

    def require(condition, message):
        if not condition:
            failures.append(message)

    for source in lock['sources']:
        path = cache/source['name']
        if not path.is_dir():
            failures.append(f'Missing source: {path}')
            continue
        try:
            def git(*cmd):
                return subprocess.check_output(['git', '-C', str(path), *cmd], text=True).strip()
            require(git('rev-parse', 'HEAD') == source['commit'], f'Commit mismatch: {path.name}')
            require(not git('status', '--porcelain'), f'Modified upstream: {path.name}')
            require(sha256(path/'LICENSE') == source['license_sha256'], f'License changed: {path.name}')
        except (OSError, subprocess.CalledProcessError) as exc:
            failures.append(f'{path.name}: {exc}')

    vocab = cache/'ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz'
    require(vocab.is_file(), 'Missing ORB vocabulary archive')
    if vocab.is_file():
        require(sha256(vocab) == lock['orb_vocabulary']['sha256'], 'ORB vocabulary hash mismatch')

    manifest = json.loads((ROOT/'dependencies'/lock['reference_manifest']).read_text())
    for entry in manifest:
        for item in entry['files']:
            path = ROOT/'dependencies/references'/entry['name']/item['path']
            require(path.is_file(), f'Missing reference: {path}')
            if path.is_file():
                require(sha256(path) == item['sha256'], f'Reference hash mismatch: {path}')

    for name in ('README.md', 'algorithm.md', 'dependencies/README.md', 'dependencies/REFERENCE_CODE.md'):
        require((ROOT/name).is_file(), f'Missing document: {name}')
    for name in ('error_correction', 'overall_flow'):
        png = ROOT/f'docs/images/{name}.png'
        require(png.is_file(), f'Missing PNG: {name}')
        if png.is_file():
            with png.open('rb') as stream:
                require(stream.read(8) == b'\x89PNG\r\n\x1a\n', f'Bad PNG header: {name}')
        try:
            ET.parse(ROOT/f'docs/images/{name}.svg')
        except (OSError, ET.ParseError) as exc:
            failures.append(f'Invalid SVG {name}: {exc}')

    if args.check_baseline:
        for item in lock['reviewed_baseline']:
            path = ROOT/item['path']
            require(path.is_file() and sha256(path) == item['sha256'], f'Reviewed baseline changed: {path}')

    if failures:
        print('\n'.join('FAIL: '+message for message in failures))
        return 1
    print('PASS: 4 pinned source repositories, reference hashes, vocabulary, documents and diagrams.')
    if args.check_baseline:
        print('PASS: Reviewed v5/common/update files remain unchanged.')
    print('This verifies preparation only; runtime build and hardware operation are not tested.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
