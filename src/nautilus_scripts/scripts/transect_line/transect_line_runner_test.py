import unittest
from transect_line_runner import Direction, process_image

class TestDirectionClassification(unittest.TestCase):

    def test_right(self):
        image1 = "./transect_images/Transect1.jpg"
        image2 = "./transect_images/Transect2.jpg"
        image3 = "./transect_images/Transect3.jpg"
        image4 = "./transect_images/Transect4.jpg"
        image5 = "./transect_images/Transect5.jpg"
        image14 = "./transect_images/Transect14.jpg"
        cur_direction = Direction.RIGHT
        self.assertEqual(process_image(image1, cur_direction), Direction.RIGHT)
        self.assertEqual(process_image(image2, cur_direction), Direction.RIGHT)
        self.assertEqual(process_image(image3, cur_direction), Direction.RIGHT)
        self.assertEqual(process_image(image4, cur_direction), Direction.RIGHT)
        self.assertEqual(process_image(image5, cur_direction), Direction.RIGHT)
        self.assertEqual(process_image(image14, cur_direction), Direction.RIGHT)

    def test_down(self):
        image6 = "./transect_images/Transect6.jpg"
        image7 = "./transect_images/Transect7.jpg"
        image11 = "./transect_images/Transect11.jpg"
        image12 = "./transect_images/Transect12.jpg"
        image13 = "./transect_images/Transect13.jpg"
        cur_direction = Direction.DOWN
        self.assertEqual(process_image(image6, cur_direction), Direction.DOWN)
        self.assertEqual(process_image(image7, cur_direction), Direction.DOWN)
        self.assertEqual(process_image(image11, cur_direction), Direction.DOWN)
        self.assertEqual(process_image(image12, cur_direction), Direction.DOWN)
        self.assertEqual(process_image(image13, cur_direction), Direction.DOWN)

    def test_left(self):
        image8 = "./transect_images/Transect8.jpg"
        image9 = "./transect_images/Transect9.jpg"
        image10 = "./transect_images/Transect10.jpg"
        cur_direction = Direction.LEFT
        self.assertEqual(process_image(image8, cur_direction), Direction.LEFT)
        self.assertEqual(process_image(image9, cur_direction), Direction.LEFT)
        self.assertEqual(process_image(image10, cur_direction), Direction.LEFT)
    
    def test_done(self):
        image15 = "./transect_images/Transect15.jpg"
        cur_direction = Direction.RIGHT
        self.assertEqual(process_image(image15, cur_direction), Direction.DONE)
    
if __name__ == '__main__':
    unittest.main()