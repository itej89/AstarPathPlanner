import unittest
import numpy as np


class obstacle_model:
    """Class to detect obstacles from a given set of polygon vertices
    """

    def __init__(self, _obstacles) -> None:
        """Constructor takes a lists of vertices belonging to multiple polygons representing the obstacles

        Args:
            _obstacles (list of lists of tuples): [[poly1_vertices], [poly2_vertices]]
        """
        self.obstacles = _obstacles 

    
    def is_inside_obstacle(self, point):
        """Given a point, the method computes if the point lies inside any of the obstacles
        Loaded in the obstacle model

        Args:
            point (tuple): point

        Returns:
            bool: True of point lies inside any obstacle, False if point is outside 
            all the loaded obstacles
        """
        
        # lopp through each polygon
        for vertices in self.obstacles:
            is_inside_obstacle = True
            #loop through pairs of vertices and find if point lies on which side of the polygon
            if len(vertices) > 2: # if not a circle
                for i in range(len(vertices)):
                    if is_inside_obstacle:
                        
                        #If final vertex then pair it with the initial vertex
                        if i == len(vertices) - 1:
                            x2, y2 = vertices[i][0], vertices[i][1]
                            x1, y1 = vertices[0][0], vertices[0][1]
                        else:
                        #Chose vertices order so the the point on the left side of the line gives a negative value
                            x2, y2 = vertices[i][0], vertices[i][1]
                            x1, y1 = vertices[i+1][0], vertices[i+1][1]

                        #Compute line coefficients for the vetex pair (ax+by+c = 0)
                        a = (y1 - y2)
                        b = (x1 - x2) * -1
                        c = x1*y2 - x2*y1
                        
                        #substitute the point and compute the direction
                        l = a*point[0] + b*point[1] + c

                        #if vertex lies outside a polygon line then break the check
                        if  l > 0:
                            is_inside_obstacle = False
                            break
            
            else: # if circle
                l = (point[0] - vertices[0][0]) ** 2 + (point[1] - vertices[0][1]) ** 2 - vertices[1] ** 2

                if l > 0:
                    is_inside_obstacle = False                    

            #if the point is found inside any polygon return true
            if is_inside_obstacle:
                return True
        
        #if the point lies outside all the poygons return false
        return False


#test case to check the obstacle_model class
class obstacle_model_test(unittest.TestCase):
    def setUp(self):
        self.obstacleModels = obstacle_model([ [(60, 0), (60, 50), (50,50), (50,0) ] ])

    def test_free_space(self):
        self.assertEqual(self.obstacleModels.check_obstacle((70,70)), False, "free_space error!!")
    
    def test_obstacle(self):
        self.assertEqual(self.obstacleModels.check_obstacle((55,40)), True, "obstacle test failed!!")
    
    def tearDown(self):
        pass

if __name__ == '__main__':
    unittest.main()