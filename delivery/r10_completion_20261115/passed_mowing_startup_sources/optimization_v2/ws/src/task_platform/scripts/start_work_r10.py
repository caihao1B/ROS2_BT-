"""BT remains authoritative while a tool action is being admitted and spun up."""
import time

def working_feedback_ready(state,now,key):
    return bool(state and state.get('active') and state.get('key')==key and
                state.get('working_confirmed') and -.03<=now-state.get('t',-100.)<.15)
def startup_interrupt(bt,now):
    if not bt or not -.03 <= now-bt.get('t',-100.) < .15:
        return 'BT_STALE'
    if bt.get('action') != 'EXECUTE':
        return bt.get('reason_code') or bt.get('action') or 'BT_NOT_EXECUTING'
    return None

def wait_startup(tick,now,directive,ready,seconds=8.):
    """Cancellation has priority even when working confirmation arrives together."""
    start=now();wall=time.monotonic()
    while True:
        tick()
        reason=startup_interrupt(directive(),now())
        if reason:return reason
        if ready():return None
        if now()-start>seconds or time.monotonic()-wall>max(20.,seconds*4):raise TimeoutError('TOOL_START_DEADLINE')
