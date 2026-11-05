import copy
import json
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from generate import build, world
from control import SimHardware

class ModelTests(unittest.TestCase):
    def setUp(self): self.p=json.loads((ROOT/'parameters.json').read_text())
    def test_joint_tree_and_mass(self):
        m=build(self.p).find('model'); links={l.get('name') for l in m.findall('link')}
        joints=m.findall('joint'); self.assertEqual(len(joints),6)
        children=[]
        for j in joints:
            self.assertIn(j.findtext('parent'),links); self.assertIn(j.findtext('child'),links)
            children.append(j.findtext('child'))
        self.assertEqual(len(set(children)),len(links)-1)
        for l in m.findall('link'):
            self.assertGreater(float(l.findtext('inertial/mass')),0)
            for axis in ('ixx','iyy','izz'): self.assertGreater(float(l.findtext('inertial/inertia/'+axis)),0)
    def test_drive_and_camera(self):
        m=build(self.p).find('model'); d=m.find("plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual(len(d.findall('left_joint')),2); self.assertEqual(len(d.findall('right_joint')),2)
        self.assertEqual(float(d.findtext('wheel_radius')),self.p['wheel_radius'])
        self.assertEqual(len(m.findall('.//sensor')),2)
        # No fictional encoders/IMU are added to physical inventory.
        self.assertIsNone(m.find(".//sensor[@type='imu']"))
    def test_reject_bad_dimensions(self):
        for key,value in [('wheel_radius',-1),('base_mass',float('nan')),('track_width',0.02),('deck_height',0.01)]:
            p=copy.deepcopy(self.p); p[key]=value
            with self.assertRaises(ValueError): build(p)
    def test_contact_height(self):
        for l in build(self.p).findall('./model/link'):
            if l.get('name').endswith('_wheel'):
                self.assertAlmostEqual(float(l.findtext('pose').split()[2])-self.p['wheel_radius'],0)
    def test_world_is_local(self):
        self.assertEqual(world().findtext('.//include/uri'),'model://omega1')
    def test_watchdog_clamps_and_rejects_nan(self):
        sent=[]; now=[0.0]; hw=SimHardware(self.p,lambda *x:sent.append(x),lambda:now[0])
        self.assertEqual(hw.drive(100,-100),(0.3,-1.5))
        self.assertEqual(hw.servo('pan',100),self.p['pan_limit'])
        with self.assertRaises(ValueError): hw.drive(math.nan,0)
        with self.assertRaises(ValueError): hw.servo('roll',0)
        now[0]=1; hw.tick(); self.assertIn('x: 0',sent[-1][2]); self.assertFalse(hw.moving)

if __name__=='__main__': unittest.main()
