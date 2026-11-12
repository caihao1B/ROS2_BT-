"""Reserved D1/D2 advisory interfaces: no fabricated implementation or plan mutation."""
import hashlib,json,math
def digest(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def request(kind,snapshot,request_id,deadline):
    if kind not in ('D1','D2') or type(request_id) is not str or not request_id or type(deadline) not in (int,float) or not math.isfinite(deadline):raise ValueError('INVALID_REPAIR_REQUEST')
    required={'mission_id','plan_revision','map_revision','geometry_sha256','remaining_segment_ids','coverage_ledger_revision','blocked_region_revision'}
    if type(snapshot) is not dict or set(snapshot)!=required:raise ValueError('INVALID_REPAIR_SNAPSHOT')
    ids=snapshot['remaining_segment_ids']
    if type(ids) is not list or not all(type(i) is str and i for i in ids) or len(set(ids))!=len(ids):raise ValueError('INVALID_REPAIR_SEGMENT_IDS')
    if any(snapshot[k] is None for k in required):raise ValueError('MISSING_REPAIR_BINDING')
    return dict(schema='DINO-REPAIR-REQUEST/1.0',kind=kind,request_id=request_id,deadline=deadline,snapshot=snapshot,snapshot_sha256=digest(snapshot))
def unavailable(req):return dict(schema='DINO-REPAIR-RESULT/1.0',request_id=req['request_id'],snapshot_sha256=req['snapshot_sha256'],status='NOT_IMPLEMENTED',proposal=None)
def validate_reply(req,result,now,current_snapshot):
    if req.get('schema')!='DINO-REPAIR-REQUEST/1.0' or result.get('schema')!='DINO-REPAIR-RESULT/1.0':raise ValueError('REPAIR_SCHEMA_UNSUPPORTED')
    if not math.isfinite(now):raise ValueError('INVALID_REPAIR_TIME')
    if now>req['deadline']:raise ValueError('REPAIR_DEADLINE_EXCEEDED')
    if digest(current_snapshot)!=req['snapshot_sha256'] or result.get('snapshot_sha256')!=req['snapshot_sha256'] or result.get('request_id')!=req['request_id']:raise ValueError('STALE_REPAIR_RESULT')
    if result.get('status')!='OK':raise ValueError('REPAIR_NOT_EXECUTABLE:'+str(result.get('status')))
    # A future plugin must validate paths/coverage and atomically commit; no bypass today.
    raise ValueError('REPAIR_COMMIT_NOT_IMPLEMENTED')
