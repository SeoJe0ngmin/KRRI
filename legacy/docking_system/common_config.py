"""One authoritative field configuration for all four estimator versions.

Version configs are generated snapshots with a live source descriptor. They
are not an editable second source. Plain external configs (e.g. run captures)
remain self-contained. This module never opens sensors or CAN.
"""
import copy
import json
import os
from pathlib import Path
import tempfile

SCHEMA = 'docking_system_common_v1'
METHODS = {
    'current': (True, True, 'Original robust_docking current configuration'),
    'a_baseline': (False, False, 'VO + gyro heading + direct tag anchor'),
    'b_tag_fusion': (True, False, 'VO + gyro heading + weighted tag correction'),
    'c_gyro_vo': (True, True, 'Gyro-aided VO + gyro heading + weighted tag correction'),
}
TASKS = {'kw_wall_stop': 'wall_stop', 'kw_park_left_tag': 'cone_entry'}


DOCUMENT_SCHEMA = 'docking_system_common_v2'


def _decode_field(node, path):
    if not isinstance(node, dict) or not isinstance(node.get('description'), str):
        raise ValueError('described config field needs description: '+path)
    if set(node) == {'description', 'value'}:
        return node['value']
    if set(node) == {'description', 'fields'} and isinstance(node['fields'], dict):
        return {key: _decode_field(child, path+'.'+key) for key, child in node['fields'].items()}
    raise ValueError('described config field needs value OR fields: '+path)


def decode(text):
    """Read standard JSON, unwrapping the documented common format if present.

    Legacy plain common configs remain readable during an incremental update.
    No non-JSON comment syntax is accepted.
    """
    doc = json.loads(text)
    if isinstance(doc, dict) and doc.get('schema') == DOCUMENT_SCHEMA:
        if not isinstance(doc.get('description'), str):
            raise ValueError('common config needs a description')
        return {'schema': SCHEMA, **{key: _decode_field(node, key) for key, node in doc.items()
                                    if key not in ('schema', 'description')}}
    return doc


def encode(doc, previous=None, descriptions=None):
    """Save standard JSON while retaining descriptions and opaque value objects."""
    notes = dict(descriptions or {})
    value_paths = set()
    heading = '공통 설정: value를 수정하고, 묶음 안의 항목은 fields에서 찾으세요. description은 설명입니다.'
    if previous:
        old = json.loads(previous)
        if old.get('schema') == DOCUMENT_SCHEMA:
            decode(previous)  # Never silently repair/drop malformed input.
            heading = old['description']
            def collect(node, path):
                notes[path] = node['description']
                if 'value' in node:
                    value_paths.add(path)
                else:
                    for key, child in node['fields'].items():
                        collect(child, path+(key,))
            for key, node in old.items():
                if key not in ('schema', 'description'):
                    collect(node, (key,))
    def render(value, path):
        description = notes.get(path, '자동 측정·점검 결과' if 'automatic' in path or path[-1] == 'mount_measurement' else '설정값')
        node = {'description': description}
        # New reports are stored intact. A previously null value such as
        # camera_model also remains a value when calibration fills it.
        report = path[0] == 'automatic' and len(path) > 1
        if isinstance(value, dict) and path not in value_paths and not report:
            node['fields'] = {key: render(child, path+(key,)) for key, child in value.items()}
        else:
            node['value'] = value
        return node
    structured = {'schema': DOCUMENT_SCHEMA, 'description': heading}
    structured.update({key: render(value, (key,)) for key, value in doc.items() if key != 'schema'})
    return json.dumps(structured, ensure_ascii=False, indent=2, allow_nan=False)+'\n'


def merge(base, overlay):
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        result[key] = merge(result[key], value) if isinstance(value, dict) and isinstance(result.get(key), dict) else copy.deepcopy(value)
    return result


def read(path):
    doc = decode(Path(path).read_text(encoding='utf-8'))
    if doc.get('schema') != SCHEMA:
        raise ValueError('unsupported common config schema: '+str(path))
    for key in ('values', 'sites', 'measurement', 'platforms', 'automatic'):
        if not isinstance(doc.get(key), dict):
            raise ValueError('common config needs an object: '+key)
    # Reject NaN/Infinity even when the permissive JSON parser accepts them.
    json.dumps(doc, allow_nan=False)
    return doc


def expand(doc, spec):
    """Resolve only runtime values; reports/instructions never enter policies."""
    from docking.configuration import resolve_config
    from docking.profiles import build_trial
    if spec['mode'] == 'site':
        return copy.deepcopy(doc['sites'][spec['site']])
    if spec['mode'] not in ('hardware', 'measure'):
        raise ValueError('unknown shared config mode')
    if spec.get('platform') not in ('nano', 'vm'):
        raise ValueError('unknown shared config platform')
    cfg = merge(doc['values'], doc['platforms'][spec['platform']])
    site = spec.get('site')
    if site:
        cfg = merge(cfg, doc['sites'][site])
    sensor = cfg['sensor']
    # These are derived values, never independently entered/cached geometry.
    cfg['body']['front_m'] = None
    sensor['calibration']['T_body_camera'] = None
    sensor['calibration']['T_dock_tag'] = None
    fusion, gyro, description = METHODS[spec['variant']]
    sensor['tag_fusion']['enabled'] = fusion
    sensor['odom']['gyro_aided_tracking'] = gyro
    experiment = dict(variant=spec['variant'], description=description, full_vio=False)
    if spec['mode'] == 'measure':
        cfg = merge(sensor, doc['measurement'])
        cfg['experiment'] = experiment
        return resolve_config(cfg)
    cfg['experiment'] = experiment
    cfg = resolve_config(cfg)
    if site in TASKS:
        cfg = build_trial(cfg, TASKS[site])
    return cfg


def load(path, snapshot=None):
    path = Path(path)
    snapshot = json.loads(path.read_text()) if snapshot is None else snapshot
    spec = snapshot.get('_shared')
    if spec is None:
        return snapshot
    source = (path.parent/spec['file']).resolve()
    # A missing shared source is an error, not permission to use stale values.
    return expand(read(source), spec)


def descriptors(system):
    system = Path(system)
    for n in range(1, 5):
        project = system/f'robust_docking_v{n}'
        version = json.loads((project/'version.json').read_text())
        for path in sorted((project/'configs').rglob('*.json')):
            cfg = json.loads(path.read_text())
            if '_shared' in cfg:
                yield path, cfg['_shared']


def check_document(system, doc):
    """Preflight every supported combination before publishing a new source."""
    if doc.get('schema') != SCHEMA:
        raise ValueError('invalid common config schema')
    json.dumps(doc, allow_nan=False)
    for path, spec in descriptors(system):
        expand(doc, spec)


def atomic(path, data):
    path = Path(path)
    fd, name = tempfile.mkstemp(prefix='.'+path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            if path.exists():
                os.fchmod(stream.fileno(), path.stat().st_mode & 0o777)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def commit(path, doc, expected):
    path = Path(path)
    check_document(path.parent.parent, doc)
    if path.read_bytes() != expected:
        raise ValueError('common config changed during measurement; retry with the new settings')
    # Only one authoritative file is replaced. All four readers see the same
    # complete old or new document, without a multi-file partial update.
    atomic(path, encode(doc, previous=expected).encode('utf-8'))


def sync(system):
    """Refresh inspectable snapshots; runtime also reads common.json directly."""
    system = Path(system)
    source = system/'configs/common.json'
    doc = read(source)
    check_document(system, doc)
    pending = []
    for path, spec in descriptors(system):
        snapshot = {'_shared': spec, **expand(doc, spec)}
        pending.append((path, (json.dumps(snapshot, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()))
    for path, data in pending:
        atomic(path, data)
    return [str(path) for path, _ in pending]


def apply_mount(path, report, lateral, expected=None):
    """Called only after mount_calibration's successful-report validation."""
    from docking.mount_calibration import _updated_config
    path = Path(path)
    before = path.read_bytes() if expected is None else expected
    doc = decode(before)
    doc['values'] = _updated_config(doc['values'], report, lateral)
    doc['values']['body']['front_m'] = None
    doc['values']['sensor']['calibration']['T_body_camera'] = None
    doc['values']['sensor']['calibration']['T_dock_tag'] = None
    doc['automatic']['mount'] = copy.deepcopy(report)
    commit(path, doc, before)
    return [str(path)]


def record_check(path, name, report, expected):
    """Diagnostics are records, not certified bounds or startup IMU biases."""
    doc = decode(expected)
    doc['automatic'][name] = copy.deepcopy(report)
    commit(path, doc, expected)
