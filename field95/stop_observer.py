"""Causal, bounded evaluation of measured stop. Never publishes commands."""
from collections import deque
import math


class StopObserver:
    def __init__(self,capacity=256):
        self.rotors=deque(maxlen=capacity);self.odoms=deque(maxlen=capacity)
        self.nav=None;self.last_nonzero_command_wall=-1e9;self.holding=False;self.violations=[];self.last_clock=None;self.rewinds=0
    def clock(self,stamp):
        if self.last_clock is not None and stamp<self.last_clock-1e-8:
            self.rotors.clear();self.odoms.clear();self.nav=None;self.rewinds+=1
        self.last_clock=stamp
    def rotor(self,stamp,velocity,wall):
        self._finite(stamp,velocity,wall);self.rotors.append((stamp,wall,velocity))
        if self.holding and abs(velocity)>.1:self._violation('ROTOR_MOVING',stamp,wall,velocity)
    def odom(self,stamp,v,w,wall):
        self._finite(stamp,v,w,wall);self.odoms.append((stamp,wall,v,w))
        if self.holding and (abs(v)>=.02 or abs(w)>=.02):self._violation('VEHICLE_MOVING',stamp,wall,[v,w])
    def navigation(self,row,wall):
        self.nav=(dict(row),wall)
        if self.holding and row.get('armed') is True:self._violation('NAVIGATION_REARMED',row.get('sim'),wall,True)
    def command(self,v,w,wall):
        self._finite(v,w,wall)
        if abs(v)>.002 or abs(w)>.002:self.last_nonzero_command_wall=wall
        if self.holding and (abs(v)>.002 or abs(w)>.002):self._violation('NONZERO_COMMAND',self.last_clock,wall,[v,w])
    def begin_hold(self):self.holding=True;self.violations=[]
    def _violation(self,reason,stamp,wall,value):
        if len(self.violations)<32:self.violations.append(dict(reason=reason,stamp=stamp,wall=wall,value=value))
    @staticmethod
    def _finite(*values):
        if not all(math.isfinite(v) for v in values):raise ValueError('STOP_OBSERVER_NONFINITE')
    @staticmethod
    def _causal(rows,sim,wall,sim_age):
        eligible=[r for r in rows if 0<=sim-r[0]<=sim_age and 0<=wall-r[1]<=.4]
        return max(eligible,key=lambda r:r[0]) if eligible else None
    def status(self,sim,wall):
        self._finite(sim,wall)
        rotor=self._causal(self.rotors,sim,wall,.15);odom=self._causal(self.odoms,sim,wall,.3)
        reason='CONFIRMED_STOPPED'
        if rotor is None:reason='WAIT_CAUSAL_FRESH_ROTOR'
        elif odom is None:reason='WAIT_CAUSAL_FRESH_ODOM'
        elif self.nav is None or not 0<=wall-self.nav[1]<=.4:reason='WAIT_FRESH_NAVIGATION'
        elif self.nav[0].get('armed') is not False:reason='NAVIGATION_NOT_DISARMED'
        elif any(not isinstance(self.nav[0].get(k),(int,float)) or not math.isfinite(self.nav[0][k]) or abs(self.nav[0][k])>.002 for k in ('v','w')):reason='ARBITER_STILL_BRAKING'
        elif wall-self.last_nonzero_command_wall<.15:reason='WAIT_COMMAND_QUIET'
        elif abs(rotor[2])>.1:reason='ROTOR_NOT_STOPPED'
        elif abs(odom[2])>=.02 or abs(odom[3])>=.02:reason='VEHICLE_NOT_STOPPED'
        return dict(stopped=reason=='CONFIRMED_STOPPED',reason=reason,rotor=list(rotor) if rotor else None,
                    odom=list(odom) if odom else None,future_rotor_pending=any(x[0]>sim for x in self.rotors),
                    violations=list(self.violations),rewinds=self.rewinds,decision_sim_s=sim,decision_wall_s=wall,
                    observer_clock_s=self.last_clock,nav_receipt_wall_s=self.nav[1] if self.nav else None,
                    navigation={k:self.nav[0].get(k) for k in ('sim','armed','v','w')} if self.nav else None,
                    last_nonzero_command_wall_s=self.last_nonzero_command_wall)
