"""R10 transport heartbeat uses the same time domain as its wall-time watchdog."""
from pathlib import Path

def derive(source):
    marker='n.create_timer(.05,publish)'
    if source.count(marker)!=1:raise ValueError('LEDGER_HEARTBEAT_ANCHOR')
    source=source.replace(marker,'n.create_timer(.05,publish,clock=_SteadyClock(clock_type=_ClockType.STEADY_TIME))')
    # Keep simulation timestamps, pairing, geometry, transactions and guards intact.
    return ('from rclpy.clock import Clock as _SteadyClock\n'
            'from rclpy.clock_type import ClockType as _ClockType\n'+source)

if __name__=='__main__':
    source=Path(__file__).with_name('ledger_r09.py')
    exec(compile(derive(source.read_text()),str(source),'exec'),globals())
