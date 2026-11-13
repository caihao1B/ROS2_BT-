"""Loopback-only, read-only operations display. Never publishes robot commands."""
from common_showcase import *
import argparse
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

def main():
    ap=argparse.ArgumentParser();group=ap.add_mutually_exclusive_group(required=True);group.add_argument('--run',type=Path);group.add_argument('--latest',action='store_true');ap.add_argument('--port',type=int,default=8768);a=ap.parse_args()
    def selected():
        run=(ROOT/read(OUT/'current.json')['run']).resolve() if a.latest else a.run.resolve()
        if not run.is_relative_to(BASE/'results/S17'):raise ValueError('EXPECTED_S17_CASE_DIRECTORY')
        return run
    page=(PKG/'scripts/dashboard.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            if self.path=='/':data=page;mime='text/html; charset=utf-8'
            elif self.path=='/state':
                try:run=selected()
                except (OSError,ValueError):self.send_error(503,'No current run');return
                out=dict(run=run.name,wall_time=time.time(),scope='独立演示 · 不计入 S17 正式验收')
                for key,path in [('monitor',run/'client/motion_monitor.json'),('mission',run/'client/mission.json'),('spec',run/'spec.json'),('metrics',run/'metrics17.json')]:
                    try:out[key]=read(path)
                    except (OSError,ValueError):out[key]=None
                monitor=run/'client/motion_monitor.json';out['update_age_wall_s']=time.time()-monitor.stat().st_mtime if monitor.exists() else None
                data=json.dumps(out,ensure_ascii=False).encode();mime='application/json; charset=utf-8'
            else:self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    print(f'http://127.0.0.1:{a.port}  '+('follow current run' if a.latest else str(a.run)),flush=True)
    try:ThreadingHTTPServer(('127.0.0.1',a.port),Handler).serve_forever()
    except KeyboardInterrupt:pass
if __name__=='__main__':main()
