#!/usr/bin/env python3
"""Cuddle-Fish-inspired helium envelope with two revolute wings and prismatic ballast."""
from pathlib import Path
import argparse,math
ROOT=Path(__file__).resolve().parents[1]
AIR=1.225;HELIUM=.164
RADII=(.5,.225,.165)
VOLUME=4/3*math.pi*math.prod(RADII)
# Trim assembly mass to the mass of displaced air; helium is included in total.
TOTAL_MASS=AIR*VOLUME;DRY_MASS=(AIR-HELIUM)*VOLUME
WING_MASS=.003;BALLAST_MASS=.006
BODY_MASS=TOTAL_MASS-2*WING_MASS-BALLAST_MASS

def material(c):return f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>'
def visual(name,geometry,color,pose='0 0 0 0 0 0'):
    return f'<visual name="{name}"><pose>{pose}</pose><geometry>{geometry}</geometry>{material(color)}</visual>'
def inertia(m,xx,yy,zz,pose='0 0 0 0 0 0'):
    return f'<inertial><pose>{pose}</pose><mass>{m}</mass><inertia><ixx>{xx}</ixx><iyy>{yy}</iyy><izz>{zz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
def box(name,xyz,size,color):
    g=f'<box><size>{size}</size></box>'
    return f'<model name="{name}"><static>true</static><pose>{xyz} 0 0 0</pose><link name="link"><collision name="collision"><geometry>{g}</geometry></collision>{visual("visual",g,color)}</link></model>'
def meshes():
    for side,sign in [('left',1),('right',-1)]:
        # Thin double-sided membrane, finite thickness for consistent rendering.
        pts=[(.12,0),(-.13,0),(-.09,sign*.165),(.02,sign*.165)]
        vertices=[(x,y,z) for z in (-.001,.001) for x,y in pts]
        faces=[(1,3,2),(1,4,3),(5,6,7),(5,7,8),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,4,8),(3,8,7),(4,1,5),(4,5,8)]
        path=ROOT/f'assets/{side}_wing.obj';path.write_text('\n'.join(['# Wing membrane in metres']+[f'v {x} {y} {z}' for x,y,z in vertices]+['f '+' '.join(map(str,f)) for f in faces])+'\n')
    # Unactuated forked tail in a horizontal plane, behind the balloon.
    path=ROOT/'assets/tail.obj';path.write_text('v -.45 0 0\nv -.70 .14 0\nv -.62 0 0\nv -.70 -.14 0\nf 1 2 3\nf 1 3 4\nf 3 2 1\nf 4 3 1\n')
def generate(factor=1):
    meshes()
    room=box('floor','0 0 -.1','10 8 .2','.24 .29 .34 1')
    for name,xyz,size in [('north','0 3.9 1.8','10 .2 3.6'),('south','0 -3.9 1.8','10 .2 3.6'),('east','4.9 0 1.8','.2 8 3.6'),('west','-4.9 0 1.8','.2 8 3.6')]:
        room+=box('wall_'+name,xyz,size,'.48 .54 .58 1')
    room+=box('table','0 1.65 .6','1.7 .9 1.2','.62 .43 .24 1')
    room+=box('cabinet','2.65 -1.55 1.2','.9 1.4 2.4','.32 .41 .48 1')
    wings=''
    for side,sign in [('left',1),('right',-1)]:
        wings+=f'''<link name="{side}_wing"><pose>0 {sign*.225} 0 0 0 0</pose>
{inertia(WING_MASS,.000012,.000012,.000022,f'-.025 {sign*.07} 0 0 0 0')}
{visual('membrane',f'<mesh><uri>file://{ROOT}/assets/{side}_wing.obj</uri></mesh>','.85 .94 .96 1')}
</link>
<joint name="{side}_hinge" type="revolute"><parent>body</parent><child>{side}_wing</child>
<axis><xyz>{sign} 0 0</xyz><limit><lower>-1.5707963</lower><upper>.17453293</upper><effort>.02</effort><velocity>10</velocity></limit><dynamics><damping>.00002</damping></dynamics></axis></joint>'''
    x,y,z=RADII
    bodyInertia=inertia(BODY_MASS,BODY_MASS*(y*y+z*z)/5,BODY_MASS*(x*x+z*z)/5,BODY_MASS*(x*x+y*y)/5,'0 0 -.018 0 0 0')
    envelope='<ellipsoid><radii>.5 .225 .165</radii></ellipsoid>'
    return f'''<?xml version="1.0"?>
<sdf version="1.12"><world name="companion_room">
<physics name="physics" type="ignored"><max_step_size>.002</max_step_size><real_time_factor>{factor}</real_time_factor></physics><gravity>0 0 -9.81</gravity>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<scene><ambient>.85 .85 .85 1</ambient><background>.1 .13 .17 1</background><shadows>false</shadows></scene>
<light type="directional" name="sun"><pose>0 0 5 0 0 0</pose><diffuse>.8 .8 .8 1</diffuse><direction>-.2 -.4 -1</direction></light>
{room}
<model name="companion"><pose>-2 0 1.6 0 0 0</pose><self_collide>false</self_collide>
<plugin filename="Companion" name="companion::Companion"/>
<link name="body">{bodyInertia}
<collision name="envelope_collision"><geometry>{envelope}</geometry><surface><friction><ode><mu>.2</mu><mu2>.2</mu2></ode></friction></surface></collision>
{visual('helium_envelope',envelope,'.72 .89 .94 1')}
{visual('tail',f'<mesh><uri>file://{ROOT}/assets/tail.obj</uri></mesh>','.85 .94 .96 1')}
{visual('gondola','<box><size>.12 .08 .04</size></box>','.1 .2 .27 1','0 0 -.155 0 0 0')}
{visual('slider_rail','<box><size>.26 .008 .008</size></box>','.25 .35 .4 1','0 0 -.055 0 0 0')}
</link>
{wings}
<link name="ballast"><pose>0 0 -.055 0 0 0</pose>{inertia(BALLAST_MASS,.000001,.000001,.000001)}
{visual('moving_weight','<box><size>.035 .035 .025</size></box>','.96 .65 .23 1')}
</link>
<joint name="ballast_slide" type="prismatic"><parent>body</parent><child>ballast</child><axis><xyz>1 0 0</xyz><limit><lower>-.11</lower><upper>.11</upper><effort>.08</effort><velocity>.2</velocity></limit><dynamics><damping>.01</damping></dynamics></axis></joint>
</model></world></sdf>'''
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--factor',type=float,default=1);p.add_argument('--output',default=str(ROOT/'worlds/companion.sdf'));a=p.parse_args()
    if not math.isfinite(a.factor) or a.factor<=0:p.error('factor must be positive and finite')
    path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(generate(a.factor));print(path)
