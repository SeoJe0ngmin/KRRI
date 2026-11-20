"""Restore only on a matching Jetson. Never starts the camera/CAN/vehicle."""
from pathlib import Path
import argparse,hashlib,json,os,platform,shlex,shutil,stat,subprocess,sys
B=Path(__file__).resolve().parent
A=B/'app/main_rec_v4_portable_20261012_044639_212337'
S=json.loads((B/'SOURCE_SYSTEM.json').read_text())
def check_platform():
 assert platform.machine()==S['architecture'],'ARM64 Jetson required (not a Mac or x86 PC)'
 assert sys.version_info[:2]==(3,10),'Python 3.10 required'
 assert Path('/etc/nv_tegra_release').is_file(),'JetPack/L4T required'
 line=Path('/etc/nv_tegra_release').read_text().splitlines()[0]
 assert line==S['l4t'].splitlines()[0],'Match source L4T R36.4.7 before restoration'
 assert platform.release()==S['kernel'],'Kernel mismatch; Kvaser driver must be rebuilt for target'
def verify(repair=False):
 entries=json.loads((B/'MANIFEST.json').read_text())
 for e in entries:
  p=B/e['path'];assert p.is_relative_to(B),e['path']
  if e['type']=='symlink':
   if repair and not p.is_symlink():
    if p.exists():
     assert p.is_file(),str(p);p.unlink()
    p.symlink_to(e['target'])
   assert p.is_symlink() and os.readlink(p)==e['target'],str(p)
  else:
   assert not p.is_symlink() and p.is_file(),str(p)
   h=hashlib.sha256()
   with p.open('rb') as f:
    for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
   assert h.hexdigest()==e['sha256'],str(p)
   if repair:os.chmod(p,e['mode'])
 print('Verified original payload:',len(entries),'entries')
def system_setup():
 # Explicitly invoked on destination only; copies vendor modules and rules.
 need=json.loads((B/'NEEDED_APT.json').read_text());missing=[]
 for pkg in sorted(set(need)|{'python3-tk','gnome-terminal','python3-venv'}):
  r=subprocess.run(['dpkg-query','-W','-f=${db:Status-Status}',pkg],text=True,capture_output=True)
  if r.stdout.strip()!='installed':missing.append(pkg)
 if missing:
  subprocess.run(['sudo','apt-get','update'],check=True)
  subprocess.run(['sudo','apt-get','install','-y',*missing],check=True)
 script='''set -eu
bundle=$1
kernel=$2
backup=/var/backups/jetson-fsm-migration-$(date +%Y%m%dT%H%M%S)
mkdir -p "$backup"
mkdir -p "/lib/modules/$kernel/extra/kvaser"
for name in kvcommon leaf mhydra usbcanII; do
 target="/lib/modules/$kernel/extra/kvaser/$name.ko"
 if [ -e "$target" ]; then cp -a "$target" "$backup/"; fi
 install -m 0644 "$bundle/system/kvaser/$name.ko" "$target"
done
for source in "$bundle"/system/udev/*.rules; do
 target="/etc/udev/rules.d/$(basename "$source")"
 if [ -e "$target" ]; then cp -a "$target" "$backup/"; fi
 install -m 0644 "$source" "$target"
done
depmod -a "$kernel"
udevadm control --reload-rules
for name in kvcommon leaf mhydra usbcanII; do modprobe "$name"; done
usermod -aG plugdev,dialout "$3"
udevadm trigger --subsystem-match=usb --action=change
udevadm trigger --subsystem-match=kvaser_can --action=change
'''
 subprocess.run(['sudo','bash','-c',script,'restore',str(B),platform.release(),os.environ['USER']],check=True)
 print('Device rules/modules installed. Log out and log in to apply group membership.')
def launchers():
 (A/'jetson-software').mkdir(exist_ok=True)
 (A/'depth_cam/rec').mkdir(exist_ok=True)
 dest=Path.home()/'Desktop';dest.mkdir(exist_ok=True)
 V=B/'VENDOR_ENV.sh'
 site=str(A/'.venv-jetson/lib/python3.10/site-packages')
 py=':'.join([site,str(B/'runtime/python-user'),str(B/'runtime/python-system310'),str(B/'runtime/python-system3')])
 V.write_text('export PYTHONPATH='+shlex.quote(py)+'\nexport LD_LIBRARY_PATH='+shlex.quote(str(B/'runtime/native-libs'))+':${LD_LIBRARY_PATH:-}\nexport PYTHONDONTWRITEBYTECODE=1\n')
 for p in (B/'desktop-original').glob('Jetson-*.desktop'):
  original=p.read_text();line=next(x for x in original.splitlines() if x.startswith('Exec='))
  oldscript=Path(line.split()[-1]).name;wrapper=B/('launch-'+p.stem+'.sh')
  wrapper.write_text('#!/bin/bash\nset -euo pipefail\nsource '+shlex.quote(str(V))+'\nexec /bin/bash '+shlex.quote(str(A/oldscript))+' "$@"\n');wrapper.chmod(0o755)
  rewritten='\n'.join(('Exec=/bin/bash "'+str(wrapper)+'"' if l.startswith('Exec=') else 'Path='+str(A) if l.startswith('Path=') else l) for l in original.splitlines())+'\n'
  target=dest/p.name
  if target.exists():
   if target.read_text()==rewritten:continue
   raise RuntimeError('Existing desktop launcher preserved; rename it first: '+str(target))
  target.write_text(rewritten);target.chmod(0o755)
  subprocess.run(['gio','set',str(target),'metadata::trusted','true'],capture_output=True)
 print('Desktop launchers ready. App source and copied venv remain unchanged.')
 print('Device-free import/GPU check: bash '+str(B/'VERIFY_RUNTIME.sh'))
parser=argparse.ArgumentParser();parser.add_argument('--system',action='store_true');parser.add_argument('--verify-only',action='store_true');args=parser.parse_args()
check_platform();verify(repair=not args.verify_only)
if not args.verify_only:
 if args.system:system_setup()
 launchers()
print('No camera/CAN channel was opened; no vehicle program was started.')
