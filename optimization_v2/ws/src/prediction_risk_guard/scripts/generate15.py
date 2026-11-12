"""Exact S14/S13 source adaptation: preserve independent safety gates."""
from pathlib import Path
import sys
pkg=Path(__file__).resolve().parents[1];s14=pkg.parent/'mission_bt';nav=pkg.parent/'navigation_adapter'
def once(s,a,b):
    if s.count(a)!=1:raise ValueError('GATE_ADAPTER_MISMATCH '+a[:90])
    return s.replace(a,b)
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
s=(nav/'src/command_gate.cpp').read_text()
s=once(s,'else if(!fresh("tool",.25)||!tool.value("stopped_confirmed",false)||tool.value("tool_on_requested",true)||tool.value("active",true))reason="TOOL_NOT_STOPPED";',
       'else if(!fresh("tool",.25)||!s14::toolPermit(tool,lease))reason="TOOL_WORK_INTERLOCK";')
s=once(s,'else if(t>=c_.at("sim_deadline").get<double>()-4)reason="SIM_DEADLINE";',
       'else if(t>=c_.at("sim_deadline").get<double>()-4)reason="SIM_DEADLINE";\n  else if(fresh("prediction",.2)&&observations_["prediction"].value("hold",false))reason="PREDICTED_CONFLICT_WAIT";')
s=once(s,'std::min({.12,s.value','std::min({lease.value("work",false)?.11:.12,(fresh("prediction",.2)?std::clamp(observations_["prediction"].value("max_speed",.12),0.,.12):.12),s.value')
s=once(s,'{"/s03/encoders","encoders"}', '{"/s03/encoders","encoders"},{"/s15/risk","prediction"}')
(out/'gate15.cpp').write_text('#include "tool_permit.hpp"\n'+s)
s=(s14/'src/mission_bt.cpp').read_text();s=once(s,'#include "decision.hpp"','#include "decision15.hpp"');s=once(s,'s14::Policy policy(s.cfg)','s15::Policy policy(s.cfg)')
(out/'bt15.cpp').write_text(s)
