import numpy as np
import cv2 as cv


class ImageProcessor:
    def __init__(self):
        pass

    def test_method(self):
        return True

    # Takes in a
    def image_evaluate_features_from_file(self, img_loc):
        img = cv.imread(img_loc)
        self.image_evaluate_features_from_matrix(img)

        return True

    def image_evaluate_features_from_matrix(self, img):
        # Call mask_land_mark and post the image to the topic for consumption by the swarm controller
        self.mask_land_mark(img)
        # Call mask_path and post the image to the topic for consumption by the swarm controller

        return True

    # This method returns an image where a mask is applied only to pixels in the image that are determined to constitute a landmark
    def mask_land_mark(self, img):
        print(img)
        return True
    
    # This method returns an image where a mask is applied only to pixels in the image that are determined to constitute the path
    def mask_path(self, img):
        return True

    def mask_lines(self, img):
        img = cv.imread(img,0)
        edges = cv.Canny(img,700,750)
        return edges
        