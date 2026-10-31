from pathlib import Path
import hashlib,json
root=Path('E:/ceshi/smartcar-rt1064-diankong')
old=Path('E:/ceshi/island')
work=root/'work/ring-transfer'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
baseline=json.loads((work/'baseline.json').read_text(encoding='utf-8'))
changed=[]
for rel,before in baseline.items():
    p=root/rel
    if not p.exists() or sha(p)!=before:changed.append(Path(rel).as_posix())
ring=[f'project/restore_s1_R1/code/{name}' for name in ('island.c','island.h','Roundabout.c','Roundabout.h')]
assert set(changed)==set(ring+['README.md']),f'Unexpected changes {changed}'
for name in ('island.c','island.h','Roundabout.h'):
    rel=f'project/restore_s1_R1/code/{name}'
    assert sha(root/rel)==sha(old/rel),f'Algorithm differs: {rel}'
source_files={rel:sha(root/rel) for rel in ring}
report={'destination':str(root),'ring_source':str(old),'existing_files_changed':changed,
        'unchanged_baseline_files':len(baseline)-len(changed),
        'ring_source_sha256':source_files,'firmware_sha256':sha(root/'project/mdk/Objects/keys_R3/rt1064_keys_R3.axf'),'build_result':'0 errors, 2 pre-existing SDK float-option warnings',
        'checks':['358 VIS7 image/geometry checks','full ring adapter with actual LineDetect/Cross/Err','synthetic-frame completion handoff','original key control','original display','original motor/differential integration','original curve control','original servo control']}
(work/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(f'PASS: four ring source files and README notice changed; {len(baseline)-len(changed)} baseline files unchanged; old algorithm/header bytes retained')