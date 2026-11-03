#!/usr/bin/env python3
"""Acceptance tests against the actual articulated Gazebo helium/flapping robot."""
import argparse,json,math,time,urllib.request,statistics,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from provenance import source_info
class Lab:
    def __init__(self,url):self.url=url;self.results=[]
    def state(self):
        with urllib.request.urlopen(self.url+'/api/state',timeout=3) as r:return json.load(r)
    def cmd(self,action,**kw):
        req=urllib.request.Request(self.url+'/api/command',json.dumps(dict(action=action,**kw)).encode(),{'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=3) as r:ack=json.load(r)['id']
        start=time.monotonic()
        while time.monotonic()-start<5:
            s=self.state()
            if s.get('ack',0)>=ack:return s
            time.sleep(.02)
        raise AssertionError('command was not acknowledged')
    def sample(self,seconds):
        end=self.state()['time']+seconds;rows=[];wall=time.monotonic();last=-1
        while True:
            s=self.state()
            if s['time']!=last:rows.append(s);last=s['time']
            if s['time']>=end:break
            if time.monotonic()-wall>max(30,seconds*3):raise AssertionError('simulation stalled')
            time.sleep(.018)
        return rows
    def reset(self,**kw):self.cmd('reset',**kw);self.sample(2)
    def check(self,name,fn):
        start=time.monotonic()
        try:
            metrics=fn();self.results.append(dict(name=name,passed=True,metrics=metrics,wall_seconds=round(time.monotonic()-start,2)));print('PASS',name,metrics,flush=True)
        except Exception as e:
            self.results.append(dict(name=name,passed=False,error=str(e),wall_seconds=round(time.monotonic()-start,2)));print('FAIL',name,str(e),flush=True)
    def finish(self):
        report={'model_revision':'helium_flapping_v2','generated_at':time.strftime('%Y-%m-%dT%H:%M:%S%z'),
         'engine':'Gazebo Jetty / native articulated joints and rigid-body dynamics','source':source_info(),
         'summary':{'passed':sum(r['passed'] for r in self.results),'total':len(self.results)},'checks':self.results,
         'limitations':['Fin thrust and drag coefficients are assumed, not measured.','Forward thrust is a rectified membrane approximation driven by measured hinge velocity.','No propellers, independent vertical force actuator, person, tracking target or camera pipeline.','Rigid envelope and wings; no aeroelastic CFD, sound or battery validation.']}
        (ROOT/'reports/results.json').write_text(json.dumps(report,indent=2)+'\n');return all(r['passed'] for r in self.results)
def check(cond,message):
    if not cond:raise AssertionError(message)
def mean(rows,key,component=None):
    return statistics.mean(s[key] if component is None else s[key][component] for s in rows)
def run(l):
    def neutral():
        l.reset();rows=l.sample(30);zs=[s['position'][2] for s in rows];forces=[sum(s['wing_forces'][:2]) for s in rows[-20:]]
        drift=zs[-1]-zs[0];check(abs(drift)<.1 and max(forces)<1e-5,f'drift={drift}, residual thrust={max(forces)}')
        return {'vertical_drift_30s_m':drift,'settled_wing_force_N':max(forces),'buoyancy_N':rows[-1]['buoyancy'],'weight_N':rows[-1]['weight_force']}
    l.check('01_passive_helium_float_without_flapping',neutral)
    def gravity():
        l.reset();l.cmd('configure',buoyancy=False,enabled=False);rows=l.sample(3);z=rows[-1]['position'][2]
        check(.12<z<.3,f'floor contact z={z}');return {'floor_height_m':z}
    l.check('02_gravity_and_envelope_collision',gravity)
    def trim():
        l.reset();l.cmd('configure',trim_grams=1.5);rows=l.sample(3);d=rows[0]['position'][2]-rows[-1]['position'][2];check(d>.15,f'descent={d}');return {'downward_trim_g':1.5,'descent_m':d}
    l.check('03_passive_trim_error_causes_descent',trim)
    def swimming():
        l.reset();l.cmd('mode',mode='swim');rows=l.sample(12);dx=rows[-1]['position'][0]-rows[0]['position'][0];fl=mean(rows[20:],'wing_forces',0);fr=mean(rows[20:],'wing_forces',1)
        check(dx>.5 and fl>0 and abs(fl-fr)<.0005,f'dx={dx}, forces={fl,fr}')
        return {'forward_travel_m':dx,'mean_left_force_N':fl,'mean_right_force_N':fr,'final_speed_mps':math.hypot(*rows[-1]['velocity'][:2])}
    l.check('04_symmetric_wings_propel_forward',swimming)
    def turning(direction):
        l.reset();l.cmd('mode',mode='turn_'+direction);rows=l.sample(5);rate=mean(rows[-20:],'angular_velocity',2);check(rate*(1 if direction=='left' else -1)>.025,f'yaw rate={rate}')
        return {'mean_yaw_rate_rad_s':rate,'left_hz':rows[-1]['wing_hz'][0],'right_hz':rows[-1]['wing_hz'][1]}
    l.check('05_differential_flapping_turns_left',lambda:turning('left'))
    l.check('06_differential_flapping_turns_right',lambda:turning('right'))
    def ballast():
        l.reset();l.cmd('ballast',position=.09);rows=l.sample(10);s=rows[-1];check(s['ballast_position']>.06 and s['angles'][1]>.08,f'ballast={s["ballast_position"]}, pitch={s["angles"][1]}')
        check(max(sum(s['wing_forces'][:2]) for s in rows[-10:])<1e-6,'stationary wings produced thrust');return {'forward_ballast_m':s['ballast_position'],'nose_down_deg':s['angles'][1]*180/math.pi}
    l.check('07_internal_mass_physically_pitches_body',ballast)
    def altitude(direction):
        l.reset();l.cmd('mode',mode=direction);rows=l.sample(14);dz=rows[-1]['position'][2]-rows[0]['position'][2];sign=1 if direction=='climb' else -1
        pitch=mean(rows[-30:],'angles',1);check(dz*sign>.12 and pitch*sign<-.04,f'dz={dz}, pitch={pitch}')
        return {'altitude_change_m':dz,'nose_up_deg':-pitch*180/math.pi,'ballast_m':rows[-1]['ballast_position']}
    l.check('08_climb_by_flapping_and_pitch',lambda:altitude('climb'))
    l.check('09_descend_by_flapping_and_pitch',lambda:altitude('descend'))
    def coast():
        l.reset();l.cmd('mode',mode='swim');s=l.sample(8)[-1];speed=math.hypot(*s['velocity'][:2]);l.cmd('mode',mode='float');rows=l.sample(15);end=math.hypot(*rows[-1]['velocity'][:2]);force=max(sum(s['wing_forces'][:2]) for s in rows[-10:]);check(speed>.06 and end<speed*.2 and force<1e-6,f'speeds={speed,end}, thrust={force}')
        return {'initial_speed_mps':speed,'coasted_speed_mps':end,'settled_wing_force_N':force}
    l.check('10_stop_flapping_and_coast_under_drag',coast)
    def joint_limits():
        l.reset();l.cmd('wings',left_hz=1.5,right_hz=1.5,left_amplitude=50,right_amplitude=50);rows=l.sample(8);angles=[v for s in rows for v in s['wing_angles'][:2]]
        check(min(angles)>-math.pi/2-.035 and max(angles)<10*math.pi/180+.035,f'angles={min(angles),max(angles)}');check(max(angles)-min(angles)>1,'wings not articulating')
        return {'min_angle_deg':min(angles)*180/math.pi,'max_angle_deg':max(angles)*180/math.pi,'max_command_frequency_hz':max(s['wing_hz'][0] for s in rows)}
    l.check('11_real_wing_joints_and_sweep_limits',joint_limits)
    def coefficient():
        l.reset();l.cmd('configure',thrust_coefficient=0);l.cmd('mode',mode='swim');rows=l.sample(8);f=max(sum(s['wing_forces'][:2]) for s in rows)
        check(f==0,'hidden propulsion remains');return {'max_aerodynamic_force_N':f,'hinge_velocity_rad_s':rows[-1]['wing_velocities'][0]}
    l.check('12_zero_fin_coefficient_has_no_hidden_thrust',coefficient)
    def manual_timeout():
        l.reset();l.cmd('manual',throttle=.8,turn=0);rows=l.sample(4);s=rows[-1];check(s['status']=='gliding' and max(s['wing_amplitudes_deg'][:2])<.01,'deadman failed');return {'status':s['status'],'wing_half_amplitude_deg':s['wing_amplitudes_deg'][0]}
    l.check('13_manual_input_timeout_stops_flapping',manual_timeout)
    def obstacle():
        l.reset(x=0,y=-.6,yaw=math.pi/2);l.cmd('mode',mode='swim');rows=l.sample(20);check(any(s['status']=='gliding_obstacle_stop' for s in rows),'no proximity stop');check(max(s['position'][1] for s in rows)<.9,'crossed table boundary')
        return {'max_y_m':max(s['position'][1] for s in rows),'final_state':rows[-1]['status']}
    l.check('14_obstacle_guard_stops_wings',obstacle)
    l.reset()
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--url',default='http://127.0.0.1:8765');a=p.parse_args();lab=Lab(a.url)
    for _ in range(100):
        try:
            if lab.state().get('connected'):break
        except OSError:pass
        time.sleep(.1)
    run(lab);sys.exit(0 if lab.finish() else 1)
