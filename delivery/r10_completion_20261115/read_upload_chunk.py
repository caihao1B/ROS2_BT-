"""Emit a bounded base64 slice of an explicitly selected publication artifact."""
import sys,json,base64
from pathlib import Path
out=Path(__file__).resolve().parent
manifest=out/sys.argv[1]
if manifest.parent!=out or manifest.name not in ['prestage_blob_manifest.json','tracking_v3_blob_manifest.json','gui_precision_prestage_manifest.json','publication_source_manifest.json','manifest.json']:
    raise SystemExit('Unknown prepared publication manifest')
entry=json.loads(manifest.read_text())[int(sys.argv[2])]
offset=int(sys.argv[3]);size=int(sys.argv[4])
if offset<0 or not 0<size<=49152:raise SystemExit('Invalid bounded slice')
with Path(entry['source']).open('rb') as f:
    f.seek(offset);data=f.read(size)
print(base64.b64encode(data).decode())
