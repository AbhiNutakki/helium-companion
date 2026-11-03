#!/usr/bin/env python3
"""Launch Gazebo and a loopback-only dashboard. Ctrl+C cleans up owned processes."""
from pathlib import Path
import argparse,json,math,os,signal,subprocess,sys,threading,time,webbrowser
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from bridge import Bridge
ROOT=Path(__file__).resolve().parent
FIELDS={
 'reset':{'x','y','z','roll','pitch','yaw'},
 'configure':{'thrust_coefficient','trim_grams','wind_x','wind_y','speed','enabled','buoyancy','safety'},
 'mode':{'mode'},'goto':{'x','y','z'},'manual':{'throttle','turn','nose_up_deg'},
 'wings':{'left_hz','right_hz','left_amplitude','right_amplitude','nose_up_deg'},
 'ballast':{'position'},'push':{'fx','fy','fz','tx','ty','tz','duration'}}
BOOLS={'enabled','buoyancy','safety'}
REQUIRED={'mode':{'mode'},'goto':{'x','y'},'ballast':{'position'}}
LIMITS={
 'left_hz':(0,1.5),'right_hz':(0,1.5),'left_amplitude':(0,50),'right_amplitude':(0,50),
 'nose_up_deg':(-25,25),'position':(-.11,.11),'throttle':(0,1),'turn':(-1,1),
 'thrust_coefficient':(0,3),'trim_grams':(-5,5),'wind_x':(-.5,.5),'wind_y':(-.5,.5),
 'speed':(.03,.3),'x':(-4,4),'y':(-3,3),'roll':(-.5,.5),'pitch':(-.5,.5),
 'yaw':(-math.pi,math.pi),'duration':(.02,1),
 **{k:(-.2,.2) for k in ('fx','fy','fz')},
 **{k:(-.015,.015) for k in ('tx','ty','tz')}}
def validate(command):
    if not isinstance(command,dict) or not isinstance(command.get('action'),str) or command['action'] not in FIELDS:raise ValueError('unknown action')
    action=command['action']
    missing=REQUIRED.get(action,set())-command.keys()
    if missing:raise ValueError('missing fields: '+', '.join(sorted(missing)))
    for key,value in command.items():
        if key=='action':continue
        if key not in FIELDS[action]:raise ValueError('unknown field: '+key)
        if key in BOOLS:
            if not isinstance(value,bool):raise ValueError(key+' must be boolean')
        elif key=='mode':
            if value not in ('float','swim','turn_left','turn_right','climb','descend','manual','off'):raise ValueError('unknown mode')
        elif isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError(key+' must be a finite number')
        else:
            bounds=(.4,2.9) if key=='z' and action=='goto' else (.3,3) if key=='z' else LIMITS.get(key)
            if bounds and not bounds[0]<=value<=bounds[1]:raise ValueError(f'{key} must be between {bounds[0]} and {bounds[1]}')
    return command
class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def respond(self,status,data,content='application/json'):
        self.send_response(status);self.send_header('Content-Type',content);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def do_GET(self):
        try:
            if self.path=='/api/state':self.respond(200,json.dumps(self.server.bridge.snapshot()).encode())
            elif self.path=='/api/report':
                p=ROOT/'reports/results.json';self.respond(200,p.read_bytes() if p.exists() else b'{}')
            elif self.path in ('/','/index.html'):self.respond(200,(ROOT/'web/index.html').read_bytes(),'text/html; charset=utf-8')
            else:self.respond(404,b'Not found','text/plain')
        except (BrokenPipeError,ConnectionResetError):pass
    def do_POST(self):
        origin=self.headers.get('Origin')
        if origin and origin not in (f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}'):
            return self.respond(403,b'{"error":"origin rejected"}')
        if self.path!='/api/command':return self.respond(404,b'{}')
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=8192:raise ValueError('invalid request size')
            command=validate(json.loads(self.rfile.read(size)))
            self.respond(200,json.dumps({'id':self.server.bridge.send(command)}).encode())
        except (ValueError,TypeError,OverflowError) as e:self.respond(400,json.dumps({'error':str(e)}).encode())

def launch(args):
    if not (ROOT/'build/libCompanion.dylib').exists() and not (ROOT/'build/libCompanion.so').exists():
        raise SystemExit('Run ./scripts/setup.sh first.')
    report=ROOT/'reports';report.mkdir(exist_ok=True)
    env=os.environ.copy();env['GZ_PARTITION']=args.partition;env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(ROOT/'build')+os.pathsep+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    os.environ['GZ_PARTITION']=args.partition
    world=ROOT/f'worlds/session-{args.port}.sdf'
    subprocess.run([sys.executable,str(ROOT/'scripts/generate_world.py'),'--factor',str(args.factor),'--output',str(world)],check=True)
    # Bind first: a port conflict must not leave an orphan Gazebo process.
    http=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    logpath=report/f'server-{args.port}.log';log=open(logpath,'w');server_process=None;gui=None
    try:
        server_process=subprocess.Popen(['gz','sim','-r','-s',str(world)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        bridge=Bridge();http.bridge=bridge;threading.Thread(target=http.serve_forever,daemon=True).start()
        if args.gui:gui=subprocess.Popen(['gz','sim','-g'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        url=f'http://127.0.0.1:{args.port}';print(f'Helium Companion: {url}\nGazebo log: {logpath}\nCtrl+C to stop.',flush=True)
        if args.open:webbrowser.open(url)
        def stop(*_):raise KeyboardInterrupt
        signal.signal(signal.SIGTERM,stop)
        while server_process.poll() is None:time.sleep(.3)
        raise RuntimeError(f'Gazebo exited with {server_process.returncode}. See {logpath}')
    except KeyboardInterrupt:pass
    finally:
        if hasattr(http,'bridge'):http.shutdown()
        http.server_close()
        for p in (gui,server_process):
            if p and p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM)
                    try:p.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        log.close()
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8765);p.add_argument('--factor',type=float,default=1);p.add_argument('--partition',default='floating_companion_lab');p.add_argument('--gui',action='store_true');p.add_argument('--open',action='store_true');launch(p.parse_args())
