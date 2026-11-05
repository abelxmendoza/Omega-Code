#!/usr/bin/env python3
"""Original parameterized Omega1 SDF and OpenSCAD geometry, stdlib only."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parent

def node(parent, tag, text=None, **attrs):
    e = E.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e

def validate(p):
    for k, v in p.items():
        if isinstance(v, (int, float)) and (not math.isfinite(v) or v <= 0):
            raise ValueError(f'{k} must be finite and positive')
    if p['track_width'] <= p['chassis_width'] + p['wheel_width']:
        raise ValueError('wheels overlap chassis')
    if p['deck_height'] - p['chassis_thickness']/2 <= p['wheel_radius']:
        raise ValueError('deck must be above wheel axles')

def geometry(parent, shape, size):
    g = node(parent, 'geometry')
    s = node(g, shape)
    if shape == 'box':
        node(s, 'size', ' '.join(map(str, size)))
    else:
        node(s, 'radius', size[0]); node(s, 'length', size[1])

def visual(link, name, size, pose, color, shape='box', collision=False):
    v = node(link, 'visual', name=name)
    node(v, 'pose', pose); geometry(v, shape, size)
    m = node(v, 'material'); node(m, 'diffuse', color); node(m, 'ambient', color)
    if collision:
        c = node(link, 'collision', name=name+'_collision')
        node(c, 'pose', pose); geometry(c, shape, size)
        return c

def link(model, name, pose, mass, dims):
    l = node(model, 'link', name=name); node(l, 'pose', pose)
    i = node(l, 'inertial'); node(i, 'mass', mass)
    t = node(i, 'inertia')
    x,y,z = dims
    for key, val in [('ixx',mass*(y*y+z*z)/12),('iyy',mass*(x*x+z*z)/12),('izz',mass*(x*x+y*y)/12),('ixy',0),('ixz',0),('iyz',0)]:
        node(t,key,val)
    return l

def joint(model, name, parent, child, axis, limit=None):
    j = node(model, 'joint', name=name, type='revolute')
    node(j,'parent',parent); node(j,'child',child)
    a=node(j,'axis'); node(a,'xyz',axis)
    lim=node(a,'limit'); node(lim,'lower',-limit if limit else -1e16); node(lim,'upper',limit if limit else 1e16)
    node(lim,'effort',0.5); node(lim,'velocity',20)
    d=node(a,'dynamics'); node(d,'damping',0.01)

def plugin(parent, filename, name):
    return node(parent,'plugin',filename=filename,name='gz::sim::systems::'+name)

def build(p):
    validate(p)
    sdf=E.Element('sdf',version='1.9'); m=node(sdf,'model',name='omega1')
    node(m,'self_collide','false')
    r=p['wheel_radius']; h=p['deck_height']; length=p['chassis_length']; width=p['chassis_width']
    base=link(m,'base_link','0 0 0 0 0 0',p['base_mass'],(length,width,0.06))
    node(base.find('inertial'),'pose',f'0 0 {h} 0 0 0')
    visual(base,'main_pcb',(length,width,p['chassis_thickness']),f'0 0 {h} 0 0 0','0.09 0.08 0.12 1',collision=True)
    visual(base,'battery_holder',(0.075,0.065,0.022),f'-0.035 0 {h-0.026} 0 0 0','0.15 0.15 0.15 1',collision=True)
    visual(base,'pi4_board',(0.085,0.056,0.002),f'-0.04 0 {h+0.02} 0 0 0','0.05 0.38 0.14 1',collision=True)
    for y in [-0.018,0.018]:
        visual(base,'usb_'+str(y),(0.017,0.014,0.016),f'-0.078 {y} {h+0.029} 0 0 0','0.65 0.65 0.65 1')
    visual(base,'line_board',(0.012,0.055,0.002),f'0.095 0 0.015 0 0 0','0.04 0.04 0.04 1',collision=True)
    for idx,y in enumerate([-0.018,0,0.018]):
        visual(base,f'line_detector_{idx}',(0.005,0.004,0.003),f'0.095 {y} 0.012 0 0 0','0.3 0.1 0.15 1')
    for front,x in [('front',p['axle_spacing']/2),('rear',-p['axle_spacing']/2)]:
        for side,sign in [('left',1),('right',-1)]:
            name=front+'_'+side+'_wheel'; y=sign*p['track_width']/2
            l=link(m,name,f'{x} {y} {r} 0 0 0',p['wheel_mass'],(2*r,p['wheel_width'],2*r))
            # SDF cylinder's axis is Z; rotation puts wheel axle along Y.
            c=visual(l,'tire',(r,p['wheel_width']),'0 0 0 1.57079632679 0 0','0.035 0.035 0.035 1','cylinder',True)
            surf=node(c,'surface'); fr=node(node(surf,'friction'),'ode'); node(fr,'mu',p['tire_friction']); node(fr,'mu2',p['tire_friction'])
            visual(l,'yellow_hub',(r*0.58,p['wheel_width']+0.001),'0 0 0 1.57079632679 0 0','0.95 0.72 0.02 1','cylinder')
            visual(l,'rotation_marker',(0.004,p['wheel_width']+0.002,r*0.75),f'0 0 {r*0.3} 0 0 0','0.8 0.8 0.8 1')
            visual(base,name+'_motor',(0.04,0.022,0.022),f'{x} {sign*(width/2-0.012)} {r} 0 0 0','0.95 0.7 0.05 1')
            joint(m,name+'_joint','base_link',name,'0 1 0')
    pan=link(m,'pan_link',f'0.065 0 {h+0.025} 0 0 0',p['pan_mass'],(0.028,0.028,0.025))
    visual(pan,'pan_servo',(0.028,0.028,0.025),'0 0 0 0 0 0','0.08 0.12 0.3 1',collision=True)
    joint(m,'pan_joint','base_link','pan_link','0 0 1',p['pan_limit'])
    head=link(m,'head_link',f'0.065 0 {h+0.055} 0 0 0',p['head_mass'],(0.025,0.065,0.04))
    visual(head,'head_plate',(0.004,0.065,0.04),'0 0 0 0 0 0','0.06 0.06 0.06 1',collision=True)
    for idx,y in enumerate([-0.018,0.018]):
        visual(head,f'ultrasonic_transducer_{idx}',(0.008,0.012),f'0.008 {y} 0 0 1.57079632679 0','0.7 0.7 0.7 1','cylinder')
    visual(head,'camera_lens',(0.005,0.008),'0.008 0 0.016 0 1.57079632679 0','0.05 0.05 0.06 1','cylinder')
    joint(m,'tilt_joint','pan_link','head_link','0 1 0',p['tilt_limit'])
    cam=node(head,'sensor',name='camera',type='camera'); node(cam,'pose','0.015 0 0.016 0 0 0'); node(cam,'topic','/omega/camera/image_raw'); node(cam,'update_rate',15)
    camera=node(cam,'camera'); node(camera,'horizontal_fov',p['camera_horizontal_fov']); im=node(camera,'image'); node(im,'width',640); node(im,'height',480); node(im,'format','R8G8B8'); clip=node(camera,'clip'); node(clip,'near',0.02); node(clip,'far',20)
    sensor=node(head,'sensor',name='ultrasonic_proxy',type='gpu_lidar'); node(sensor,'pose','0.02 0 0 0 0 0'); node(sensor,'topic','/omega/sim/ultrasonic_scan'); node(sensor,'update_rate',10)
    lidar=node(sensor,'lidar'); scan=node(lidar,'scan'); hz=node(scan,'horizontal'); node(hz,'samples',9); node(hz,'resolution',1); node(hz,'min_angle',-p['ultrasonic_fov']/2); node(hz,'max_angle',p['ultrasonic_fov']/2)
    ran=node(lidar,'range'); node(ran,'min',p['ultrasonic_min']); node(ran,'max',p['ultrasonic_max']); node(ran,'resolution',0.01)
    drive=plugin(m,'gz-sim-diff-drive-system','DiffDrive')
    for side in ['left','right']:
        for front in ['front','rear']: node(drive,side+'_joint',front+'_'+side+'_wheel_joint')
    for key,value in [('wheel_separation',p['track_width']),('wheel_radius',r),('topic','/omega/sim/cmd_vel'),('odom_topic','/omega/sim/odom'),('tf_topic','/omega/sim/tf'),('frame_id','odom'),('child_frame_id','base_link'),('min_linear_velocity',-p['max_linear_speed']),('min_angular_velocity',-p['max_angular_speed']),('max_linear_velocity',p['max_linear_speed']),('max_angular_velocity',p['max_angular_speed'])]: node(drive,key,value)
    for axis in ['pan','tilt']:
        c=plugin(m,'gz-sim-joint-position-controller-system','JointPositionController'); node(c,'joint_name',axis+'_joint'); node(c,'topic','/omega/sim/'+axis); node(c,'p_gain',1); node(c,'i_gain',0); node(c,'d_gain',0.05); node(c,'cmd_max',0.2); node(c,'cmd_min',-0.2)
    plugin(m,'gz-sim-joint-state-publisher-system','JointStatePublisher')
    return sdf

def world():
    s=E.Element('sdf',version='1.9'); w=node(s,'world',name='omega1_bench')
    ph=node(w,'physics',name='default',type='ignored'); node(ph,'max_step_size',0.001); node(ph,'real_time_factor',1)
    for f,n in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        p=plugin(w,'gz-sim-'+f+'-system',n)
        if n=='Sensors': node(p,'render_engine','ogre2')
    scene=node(w,'scene'); node(scene,'ambient','0.6 0.6 0.6 1'); node(scene,'background','0.7 0.8 0.9 1')
    sun=node(w,'light',name='sun',type='directional'); node(sun,'pose','0 0 5 0 0 0'); node(sun,'direction','-0.5 0.2 -1'); node(sun,'diffuse','0.9 0.9 0.9 1'); node(sun,'cast_shadows','true')
    for name,size,pose,color in [('floor',(6,6,0.1),'0 0 -0.05 0 0 0','0.65 0.65 0.65 1'),('obstacle',(0.3,0.5,0.3),'0.9 0 0.15 0 0 0','0.8 0.25 0.1 1')]:
        model=node(w,'model',name=name); node(model,'static','true'); l=node(model,'link',name='body'); visual(l,name,size,pose,color,collision=True)
    inc=node(w,'include'); node(inc,'uri','model://omega1'); node(inc,'pose','0 0 0.002 0 0 0')
    return s

def save(tree,path):
    E.indent(tree); E.ElementTree(tree).write(path,encoding='utf-8',xml_declaration=True)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--parameters',type=Path,default=ROOT/'parameters.json'); ap.add_argument('--output',type=Path,default=ROOT/'generated'); args=ap.parse_args()
    p=json.loads(args.parameters.read_text()); model=build(p)
    target=args.output/'models'/'omega1'; target.mkdir(parents=True,exist_ok=True)
    save(model,target/'model.sdf'); save(world(),args.output/'bench.sdf')
    (target/'model.config').write_text('<model><name>Omega1 (uncalibrated)</name><version>0.1</version><sdf version="1.9">model.sdf</sdf><author><name>Omega-Code</name></author><description>Original procedural model; measurements pending.</description></model>\n')
    # Editable original solid geometry reference. Values come from the same profile.
    scad=['// Metres converted to millimetres. Approximate reference, not fabrication CAD.','$fn=48;','scale(1000) {']
    for l in model.findall('./model/link'):
        lp=l.findtext('pose','0 0 0 0 0 0').split(); xyz='['+','.join(lp[:3])+']'
        for v in l.findall('visual'):
            vp=list(map(float,v.findtext('pose','0 0 0 0 0 0').split())); angles=[x*180/math.pi for x in vp[3:]]
            prefix=f'translate({xyz}) translate({vp[:3]}) rotate({angles}) '
            b=v.find('./geometry/box/size')
            if b is not None: shape='cube(['+','.join(b.text.split())+'], center=true);'
            else:
                c=v.find('./geometry/cylinder'); shape=f'cylinder(r={c.findtext("radius")}, h={c.findtext("length")}, center=true);'
            col=v.findtext('./material/diffuse').split()[:3]
            scad.append('color(['+','.join(col)+']) '+prefix+shape)
    (args.output/'omega1.scad').write_text('\n'.join(scad+['}'])+'\n')
    (args.output/'parameters.json').write_text(json.dumps(p,indent=2)+'\n')
    print(f'Generated {target}; fidelity: {p["fidelity"]}')

if __name__=='__main__': main()
