import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulator'))
from grid_logic import *
class GridTests(unittest.TestCase):
 def setUp(self):self.g=dict(DEFAULT_GRID)
 def test_grid(self):
  self.assertEqual(cell_xy(self.g,'C3'),{'x_mm':100.,'y_mm':100.})
  self.assertEqual(len(cells(self.g)),9)
  with self.assertRaises(ValueError):cell_xy(self.g,'Z9')
 def test_rotation(self):
  self.assertEqual(steps_to_degrees(self.g,2),120)
  with self.assertRaises(ValueError):steps_to_degrees(self.g,9)
 def test_rail(self):self.assertEqual(rail_mm_to_percent(self.g,150),50)
 def test_cycle(self):
  p={'base':90,'shoulder':90,'elbow':90,'gripper':35,'rail':20,'grip_closed':False}
  q={**p,'rail':80}
  frames=cycle_frames(p,q,p,p,self.g)
  self.assertEqual(len(frames),8)
  self.assertTrue(frames[1][1]['grip_closed'])
  self.assertFalse(frames[-1][1]['grip_closed'])
 def test_invalid_pose(self):
  with self.assertRaises(ValueError):validate_pose({'base':190},self.g)
if __name__=='__main__':unittest.main()
