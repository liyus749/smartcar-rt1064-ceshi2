from pathlib import Path
import hashlib,json,shutil
root=Path('E:/ceshi/smartcar-rt1064-diankong')
source=Path('E:/ceshi/island')
work=root/'work/ring-transfer'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
# Record the untouched electrical baseline before editing any active file.
def excluded(p):
    rel=p.relative_to(root)
    return '.git' in rel.parts or rel.parts[0]=='work' or 'Objects' in rel.parts or 'Listings' in rel.parts or p.name.endswith(('.uvguix.Admin','.tmp'))
manifest={str(p.relative_to(root)):sha(p) for p in root.rglob('*') if p.is_file() and not excluded(p)}
(work/'baseline.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
files=['project/restore_s1_R1/code/island.c','project/restore_s1_R1/code/island.h',
       'project/restore_s1_R1/code/Roundabout.c','project/restore_s1_R1/code/Roundabout.h']
for rel in files:
    backup=work/'backup'/rel;backup.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(root/rel,backup)
    shutil.copy2(source/rel,root/rel)
# Electrical Err.c stays byte-for-byte unchanged. Hold RECOVER for one image if
# that final image still contains a synthetic boundary, so normal-road width
# learning in the unmodified electrical module never learns it.
p=root/'project/restore_s1_R1/code/Roundabout.c'
s=p.read_text(encoding='utf-8')
s=s.replace('static int last_stage;', 'static int last_stage;\nstatic uint8 finish_pending; /* image-thread-owned completion handoff */')
s=s.replace('reset_pending = 0; frame_age = phase_ticks = total_ticks = 0; last_stage = 0;', 'reset_pending = finish_pending = 0; frame_age = phase_ticks = total_ticks = 0; last_stage = 0;')
s=s.replace('frame_age = phase_ticks = total_ticks = 0; last_stage = 0; reset_pending = 0;', 'frame_age = phase_ticks = total_ticks = 0; last_stage = 0; finish_pending = 0; reset_pending = 0;')
s=s.replace('    mask = __get_PRIMASK(); __disable_irq(); frame_age = 0; __set_PRIMASK(mask);',
'''    if(finish_pending) { Island_State = ROUND_NORMAL; finish_pending = 0; }
    mask = __get_PRIMASK(); __disable_irq(); frame_age = 0; __set_PRIMASK(mask);''')
s=s.replace('    Island_Detect();', '''    Island_Detect();
    if(Island_State == ROUND_NORMAL)
        for(y = 0; y < MT9V03X_H; y++)
            if(island_patch_left[y] || island_patch_right[y])
            { Island_State = ROUND_RECOVER; finish_pending = 1; break; }''')
p.write_text(s,encoding='utf-8')
for name in ['island_vision_host.c','island_visual_scene_host.c','run_island_vision_tests.py','run_roundabout_adapter_tests.py']:
    assert not (root/'tests'/name).exists(),f'Unexpected existing test {name}'
    shutil.copy2(source/'tests'/name,root/'tests'/name)
print('Transferred VIS7 source and four ring tests. Original electrical baseline and ring files saved.')