import unittest
# import Drone_Control as dc
import Image_Processing as ip

class TestImageProcessing(unittest.TestCase):
    def setUp(self):
        self.imageProcessor = ip.ImageProcessor()

    def test_test1(self):
        self.assertTrue(True)
    
    def test_test2(self):
        self.assertTrue(self.imageProcessor.test_method())

if __name__ == '__main__':
    unittest.main()