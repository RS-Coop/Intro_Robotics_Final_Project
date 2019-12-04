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

    # Identify a landmark in an image
    def test_identify_landmark(self):
        fileName = "has_landmark.jpeg"
        self.assertTrue(self.imageProcessor.image_evaluate_features_from_file(fileName))

if __name__ == '__main__':
    unittest.main()