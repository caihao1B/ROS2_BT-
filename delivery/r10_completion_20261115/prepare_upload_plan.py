"""Plan an immutable Git tree overlay from the sealed publication manifest.

Run after prepare.py. No network access, authentication, or branch mutation.
"""
from pathlib import Path
import json

OUT=Path(__file__).resolve().parent
remote=json.loads((OUT/'remote_tree.json').read_text())
tree=remote['tree'] if isinstance(remote,dict) else remote
known={e['path']:e['sha'] for e in tree if e.get('type')=='blob'}
modes={e['path']:e.get('mode','100644') for e in tree if e.get('type')=='blob'}
blobs=set(known.values())
for name in ['prestage_manifest.json','prestage_blob_manifest.json',
             'tracking_v3_blob_manifest.json','prestage_tracking_v3_receipt.json',
             'prestage_tracking_v3_config_receipt.json','prestage_gui_precision_receipt.json',
             'prestage_source_complete_receipt.json']:
    d=json.loads((OUT/name).read_text())
    entries=d if isinstance(d,list) else d.get('entries',d.get('blobs',[]))
    for e in entries:
        sha=e.get('git_blob',e.get('sha_expected'))
        if not sha:raise ValueError((name,e))
        known[e['path']]=sha;blobs.add(sha)
modes['optimization_v2/three_tasks/gui.sh']='100755'
manifest=json.loads((OUT/'manifest.json').read_text())
binary=[];entries=[];used=set();changed=[]
for index,e in enumerate(manifest):
    if e['git_blob']==known.get(e['path']):continue
    changed.append(dict(index=index,path=e['path'],sha=e['git_blob'],bytes=e['bytes']))
    item=dict(path=e['path'],mode=modes.get(e['path'],'100644'),type='blob')
    if e['git_blob'] in blobs:
        item['sha']=e['git_blob']
    elif e['binary']:
        item['sha']=e['git_blob']
        if e['git_blob'] not in used:
            binary.append(dict(index=index,path=e['path'],bytes=e['bytes'],sha=e['git_blob']))
            used.add(e['git_blob'])
    else:
        item['content']=Path(e['source']).read_text()
    entries.append(item)
chunks=[];chunk=[];size=0
for e in entries:
    length=len(json.dumps(e,ensure_ascii=False).encode())
    if chunk and size+length>100_000:chunks.append(chunk);chunk=[];size=0
    chunk.append(e);size+=length
if chunk:chunks.append(chunk)
for index,chunk in enumerate(chunks):
    (OUT/f'final_tree_chunk_{index}.json').write_text(json.dumps(chunk,ensure_ascii=False))
result=dict(base_tree_sha='bea71e350784c1814ab0ffc76a20d4b016035fd3',
            changed=changed,binary=binary,chunk_count=len(chunks),
            files_in_manifest=len(manifest),
            source_files=sum(e['path'].startswith('optimization_v2/ws/src/') for e in manifest),
            new_binary_bytes=sum(e['bytes'] for e in binary),branch_updated=False)
(OUT/'upload_plan.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in result.items() if k not in ['changed','binary']}))
