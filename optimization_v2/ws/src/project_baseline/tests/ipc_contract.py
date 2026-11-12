"""Reject late/wrong-generation decisions before they can affect the sole executor."""
def rejection_reason(row,pending,current_context,now,received):
    if type(row) is not dict or row.get('schema')!='DINO-TASK-BT-RESULT/1.0':return 'SCHEMA'
    if pending is None:return 'NO_PENDING_TICK'
    if row.get('sequence')!=pending[0]:return 'SEQUENCE'
    if row.get('context')!=current_context:return 'CONTEXT_CHANGED'
    if not 0<=now-pending[1]<=.5:return 'RESPONSE_DEADLINE'
    if not 0<=now-received<.4:return 'RECEIPT_AGE'
    if type(row.get('commands')) is not list:return 'COMMAND_SCHEMA'
    return None
def accept_reply(row,pending,current_context,now,received):return rejection_reason(row,pending,current_context,now,received) is None
