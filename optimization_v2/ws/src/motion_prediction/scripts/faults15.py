"""Owned process loss injection, separate from dataset simulator binding."""
import os,signal
from common15 import *
import runtime15

def channel_loss(out,scenario='turning',method='learned',seconds=90,wall=300,gui=False):
    original=runtime15.adapt
    once=runtime15.once
    def adapted(source):
        source=original(source)
        source=once(source,"        launch('prediction',","        prediction_process=launch('prediction',")
        source=once(source,"            status=live['status']",'''            if t>=18 and 'prediction_loss' not in actions:
                actions.add('prediction_loss');prediction_process.send_signal(signal.SIGSTOP);events.append(dict(t=t,event='prediction_process_paused',pid=prediction_process.pid))
            if t>=23 and 'prediction_loss' in actions and 'prediction_restore' not in actions:
                actions.add('prediction_restore');prediction_process.send_signal(signal.SIGCONT);events.append(dict(t=t,event='prediction_process_resumed',pid=prediction_process.pid))
            status=live['status']''')
        source=once(source,"        if 'source_process' in locals() and source_process.poll() is None:source_process.send_signal(signal.SIGCONT)","        if 'prediction_process' in locals() and prediction_process.poll() is None:prediction_process.send_signal(signal.SIGCONT)\n        if 'source_process' in locals() and source_process.poll() is None:source_process.send_signal(signal.SIGCONT)")
        return source
    runtime15.adapt=adapted
    try:
        result=runtime15.run_case(out,scenario,method,'test',seconds,wall,gui,fault='none')
        result['fault']='channel_loss';dump(Path(out)/'runtime.json',result);return result
    finally:runtime15.adapt=original
