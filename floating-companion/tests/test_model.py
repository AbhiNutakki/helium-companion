import sys,math,unittest,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from generate_world import generate,TOTAL_MASS,VOLUME
from app import validate
class ModelTests(unittest.TestCase):
    def setUp(self):self.root=ET.fromstring(generate());self.model=self.root.find(".//model[@name='companion']")
    def test_only_two_wings_and_internal_slide(self):
        joints=self.model.findall('joint');self.assertEqual({j.attrib['name'] for j in joints},{'left_hinge','right_hinge','ballast_slide'});self.assertEqual([j.attrib['type'] for j in joints].count('revolute'),2)
    def test_no_tracking_entities_or_actuators(self):
        self.assertIsNone(self.root.find(".//model[@name='person_marker']"));self.assertEqual(self.model.findall('.//sensor'),[])
        links={x.attrib['name'] for x in self.model.findall('link')};self.assertEqual(links,{'body','left_wing','right_wing','ballast'})
    def test_assembly_mass_includes_every_link(self):
        actual=sum(float(x.findtext('inertial/mass')) for x in self.model.findall('link'));self.assertAlmostEqual(actual,TOTAL_MASS,places=10);self.assertAlmostEqual(actual,1.225*VOLUME,places=10)
    def test_reject_removed_propeller_controls(self):
        with self.assertRaises(ValueError):validate({'action':'configure','max_vertical':.1})
    def test_reject_tracking_commands(self):
        with self.assertRaises(ValueError):validate({'action':'mode','mode':'follow'})
    def test_reject_nonfinite(self):
        with self.assertRaises(ValueError):validate({'action':'wings','left_hz':float('nan')})
    def test_wing_controls_are_accepted(self):
        self.assertEqual(validate({'action':'wings','left_hz':1,'right_hz':1})['left_hz'],1)
    def test_reject_malformed_actions(self):
        for action in ([],{},None,42):
            with self.subTest(action=action),self.assertRaises(ValueError):validate({'action':action})
    def test_require_command_targets(self):
        for command in ({'action':'mode'},{'action':'goto','x':0},{'action':'ballast'}):
            with self.subTest(command=command),self.assertRaises(ValueError):validate(command)
    def test_reject_values_that_would_be_silently_clamped(self):
        for action,key,value in (('wings','left_hz',2),('ballast','position',.2),('configure','trim_grams',6),('push','duration',0)):
            with self.subTest(key=key),self.assertRaises(ValueError):validate({'action':action,key:value})
    def test_accept_physical_limit_endpoints(self):
        validate({'action':'wings','left_hz':0,'right_hz':1.5,'left_amplitude':0,'right_amplitude':50})
        validate({'action':'goto','x':-4,'y':3,'z':2.9})
if __name__=='__main__':unittest.main()
