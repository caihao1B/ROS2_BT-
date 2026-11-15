"""Exercise the real local browser against live and historical evidence."""
from pathlib import Path
import base64,json,time,urllib.request,urllib.error
import websocket
OUT=Path(__file__).resolve().parent/'gui_qa';OUT.mkdir(exist_ok=True)
tab=next(t for t in json.load(urllib.request.urlopen('http://127.0.0.1:9225/json')) if t['type']=='page')
wire=websocket.create_connection(tab['webSocketDebuggerUrl'],origin='http://localhost:9225',timeout=10)
sequence=0;errors=[]
def call(method,params=None):
 global sequence
 sequence+=1;wire.send(json.dumps(dict(id=sequence,method=method,params=params or {})))
 while True:
  r=json.loads(wire.recv())
  if r.get('method')=='Runtime.exceptionThrown':errors.append(r)
  if r.get('id')==sequence:
   if 'error' in r:raise RuntimeError(r['error'])
   return r.get('result',{})
def js(expression):
 result=call('Runtime.evaluate',dict(expression=expression,returnByValue=True,awaitPromise=True))
 if result.get('exceptionDetails'):raise RuntimeError(result)
 return result.get('result',{}).get('value')
def wait_js(expression,seconds=8):
 start=time.monotonic()
 while not js(expression):
  if time.monotonic()-start>seconds:raise TimeoutError(expression)
  time.sleep(.2)
def screenshot(name):
 r=call('Page.captureScreenshot',dict(format='png',captureBeyondViewport=False));(OUT/(name+'.png')).write_bytes(base64.b64decode(r['data']))
call('Runtime.enable');call('Page.enable');call('Page.reload',dict(ignoreCache=True));wait_js('state && state.available')
checks={};wait_js('document.querySelector("#runs").options.length>2');checks['initial_run_selector_loaded']=True;checks['live_run']=js('state.run.status')=='LIVE';live_run=js('state.run.id')
for name in ['overview','decision','sensors','work','raw']:
 js(f'document.querySelector("[data-tab={name}]").click();window.scrollTo(0,0)');time.sleep(.25)
 checks['tab_'+name]=js(f'document.querySelector("#{name}").classList.contains("active")')
 if name in ['overview','decision','sensors']:screenshot(name)
wait_js('state.observer.channels && state.observer.channels.lidar && state.observer.channels.camera')
checks['six_actual_ros_channels']=set(js('Object.keys(state.observer.channels)'))>=set(['imu','wheel','camera','lidar','gnss','odom'])
checks['actual_camera_image']=js('document.querySelector("#camera").naturalWidth')>0
checks['bounded_lidar_points']=0<js('state.observer.channels.lidar.sample_xyz.length')<=512
js('document.querySelector("#pause").click()');initial=js('state.sim_t');before=json.load(urllib.request.urlopen('http://127.0.0.1:8088/api/state'))['sim_t'];time.sleep(2)
after=json.load(urllib.request.urlopen('http://127.0.0.1:8088/api/state'))['sim_t'];checks['freeze_display_only']=js('state.sim_t')==initial and after>before;js('document.querySelector("#pause").click()')
historical='R12/mowing_20261115_054835_908507'
js('(()=>{let s=document.querySelector("#runs");if(!Array.from(s.options).some(o=>o.value==='+json.dumps(historical)+'))s.add(new Option("历史 V3 故障记录",'+json.dumps(historical)+'));s.value='+json.dumps(historical)+';s.dispatchEvent(new Event("change"))})()')
wait_js('state.run.id==='+json.dumps(historical));checks['historical_not_live']=js('state.run.status')=='CLOSED';checks['historical_no_cross_run_sensor']=js('Object.keys(state.observer).length')==0
checks['selected_vs_original']=js('geom.selected_count===5 && geom.original_count===47')
js('document.querySelector("[data-tab=work]").click();window.scrollTo(0,0)');screenshot('historical_work')
inspection=next(r['id'] for r in json.load(urllib.request.urlopen('http://127.0.0.1:8088/api/runs')) if r['task']=='inspection')
js('document.querySelector("#runs").value='+json.dumps(inspection)+';document.querySelector("#runs").dispatchEvent(new Event("change"))')
wait_js('state.run.id==='+json.dumps(inspection))
checks['inspection_original_route_retained']=js('geom.original_route_length_m>80 && geom.selected_route_length_m<=geom.original_route_length_m && geom.route.length>100')
checks['inspection_distance_units']=js('document.querySelector("#progress").textContent.endsWith(" m") && document.querySelector("#budget").textContent.endsWith(" m")')
js('document.querySelector("[data-tab=overview]").click();window.scrollTo(0,0)');screenshot('inspection')
failed_spray='R12/spray_20261115_060145_047614'
js('document.querySelector("#runs").value='+json.dumps(failed_spray)+';document.querySelector("#runs").dispatchEvent(new Event("change"))')
wait_js('state.run.id==='+json.dumps(failed_spray))
checks['closed_spray_error_visible']=js('state.run.status==="CLOSED" && document.querySelector("#acceptance").textContent.includes("LOCALIZATION_REVALIDATION_BOUND") && document.querySelector("#acceptance").textContent.includes("已停止")')
js('document.querySelector("[data-tab=work]").click();window.scrollTo(0,0)');screenshot('spray_residual')
js('document.querySelector("#runs").value="auto";document.querySelector("#runs").dispatchEvent(new Event("change"))');wait_js('state.run.status==="LIVE"')
call('Emulation.setDeviceMetricsOverride',dict(width=430,height=932,deviceScaleFactor=1,mobile=True))
js('document.querySelector("[data-tab=overview]").click();window.scrollTo(0,0)');time.sleep(.5)
checks['mobile_no_horizontal_overflow']=js('document.documentElement.scrollWidth<=window.innerWidth+1');screenshot('mobile')
call('Emulation.clearDeviceMetricsOverride');time.sleep(.2);screenshot('overview')
for name,url,method in [('path_rejected','http://127.0.0.1:8088/api/state?run=../../etc/passwd','GET'),('post_rejected','http://127.0.0.1:8088/api/state','POST')]:
 try:urllib.request.urlopen(urllib.request.Request(url,method=method));checks[name]=False
 except urllib.error.HTTPError as ex:checks[name]=ex.code in (400,404,405,501)
checks['browser_no_exceptions']=not errors
report=dict(passed=all(checks.values()),checks=checks,live_run=live_run,historical_run=historical,errors=errors,simulation_before_pause=before,simulation_after_pause=after,scope='Real local browser and live observed ROS data; GUI only, no task success credit')
(OUT/'browser_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False));wire.close()
if not report['passed']:raise SystemExit(2)
