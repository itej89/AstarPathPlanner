import math
import numpy as np
import cv2 as cv

import pyclipper
from obstacle_model import *

class environment:
    """Class contains all the funcitonality to create and manipulate environemnt and its visualization
    """
    
    def inflate_polygon(self, vertices, radius):
        if(len(vertices) > 2):
            pco = pyclipper.PyclipperOffset()
            pco.AddPath(vertices, pyclipper.PT_CLIP, pyclipper.ET_CLOSEDPOLYGON)
            vertices_inflated = pco.Execute(radius)
            vertices_inflated = [tuple(x) for x in vertices_inflated[0]]
        else:
            vertices_inflated = vertices.copy()
            vertices_inflated[1] += radius

        return vertices_inflated

    def __init__(self, height, width, inflation_radius, step_size, robot_wheel_radius, wheel_base_width) -> None:
        """Initialize environment parameters

        Args:
            height (int): height of the map
            width (int): width of the map
        """
        self.height = height
        self.width = width
        self.inflation_radius = inflation_radius

        self.step_size = step_size
        self.robot_wheel_radius = robot_wheel_radius
        self.wheel_base_width = wheel_base_width

        #create a map fo given dimentions. 3 channels for opencv BGR
        self.map = np.ones((height, width, 3))        

        #create obstacle models for all the objects in the environment

        #create original boundary obstacle model
        self.boundary_model = obstacle_model([
            [(600,0), (600,200), (0,200), (0,0)]
            ])
        
        #create inflated boundary obstacle model
        self.inflated_boundary_model = obstacle_model([
            self.inflate_polygon([(600,0), (600,200), (0,200), (0,0)],  -1*self.inflation_radius),
            ])
        
        #create original polygon objects obstacle model
        self.original_obstacle_model = obstacle_model([  
            [(150,200), (150,75), (165,75), (165,200), (150,200)], # Polygon corresponding to botom rectangular pillar
            [(250,125), (250,0), (265,0), (265,125), (250,125)],
            [(400,110), 50]                                        # Polygon corresponding to Top rectangular pillar
        ])      


        #create inflated polygon objects obstacle model
        self.inflated_obstacle_model = obstacle_model([
            self.inflate_polygon([(150,200), (150,75), (165,75), (165,200), (150,200)],  self.inflation_radius),
            self.inflate_polygon([(250,125), (250,0), (265,0), (265,125), (250,125)],  self.inflation_radius),
            self.inflate_polygon([(250,125), (250,0), (265,0), (265,125), (250,125)],  self.inflation_radius),
            self.inflate_polygon([(400,110), 50],  self.inflation_radius),
        ])        

    
    def create_map(self):
        """Idnetify obstacles and free space in the map uisng the obstacle models
        """
        #Iterate through all the states in the enviornement
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                #Checks if state present inside the non-inflated obstacle
                if self.original_obstacle_model.is_inside_obstacle((j,i)):
                    self.map[i,j] = [0x80, 0x80, 0xff]
                
                #Checks if state present inside an inflated obstacle
                elif self.inflated_obstacle_model.is_inside_obstacle((j,i)):
                    self.map[i,j] = [0x7a, 0x70, 0x52]

                #Checks if state present outside the inflated boundary
                elif not self.inflated_boundary_model.is_inside_obstacle((j,i)):
                    self.map[i,j] = [0x7a, 0x70, 0x52]

                #Identify as state belongs to the free space
                else:
                    self.map[i,j] = [0xc2, 0xba, 0xa3]


    def is_valid_position(self, position):
        """Checks if a given position belongs to free space or obstacle space
        Args:
            position (tuple): state of the agent to be verified
        Returns:
            bool: True if pixel belongs to free space else False
        """
        if position[0] >=0 and position[0] < 200 and position[1] >=0 and position[1] < 600:
            #Check if a state belongs to free psace by comparing the color of the respective pixel in the environment
            if  self.map[position[0], position[1]][0] == 0xc2 and \
                self.map[position[0], position[1]][1] == 0xba and \
                self.map[position[0], position[1]][2] == 0xa3:
                return True
        
        #return false of state is in obstacle space
        return False


    def refresh_map(self):
        """Refreshes map with updated image map
        """
        #Flip the map to satisfy the environment direction
        image = cv.flip(self.map.astype('uint8'), 0)
        image = cv.resize(image, (0,0), fx = 2, fy = 2)
        cv.imshow("map", image)

    def update_map(self, position):
        """Highlights a point in the environment at the given locaiton
        Args:
            position (tuple): pixel location
        """
        i, j, _ = position
        self.map[i, j] = [255, 255, 255]
        self.refresh_map()

    def update_action(self, pose, action, validity_checker):
        Xi,Yi,Thetai = pose
        UL, UR = action
        t = 0
        # r = 0.038
        # L = 0.354
        r = self.robot_wheel_radius
        L = self.wheel_base_width
        dt = 0.1
        Xn=Xi
        Yn=Yi
        Thetan = 3.14 * Thetai / 180
        # Xi, Yi,Thetai: Input point's coordinates
        # Xs, Ys: Start point coordinates for plot function
        # Xn, Yn, Thetan: End point coordintes
        cost_to_go=0
        while t<self.step_size:
            t = t + dt
            Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Xn += Delta_Xn
            Yn += Delta_Yn
            Thetan += (r / L) * (UR - UL) * dt
            cost_to_go=cost_to_go+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) *
            dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))

            if validity_checker((round(Xn), round(Yn), Thetan)):
                if self.map[round(Xn), round(Yn)][0] == 0 and self.map[round(Xn), round(Yn)][1] == 255 \
                  and self.map[round(Xn), round(Yn)][2] == 0:
                    continue
                self.map[round(Xn), round(Yn)] = [0, 255, 0]
            else:
                 break
            


    def save_image(self, file_path):
        """saves current state of the environment in the file location as image 
        Args:
            file_path (string): absolute path of the file where the image needs to be saved
        """
        #Flip the map to satisfy the environment direction
        image = cv.flip(self.map.astype('uint8'), 0)
        image = cv.resize(image, (0,0), fx = 2, fy = 2)
        cv.imwrite(file_path, image)

    def highlight_state(self, position, size, color):
        """Draws a circle at the given location in the environment
        Args:
            position (tuple): pixel location
        """
        self.map =  cv.circle(self.map, (position[1],position[0]), size, color, -1)
        self.refresh_map()

    def highlight_point(self, position):
        """Highlights a point in the environment at the given locaiton
        Args:
            position (tuple): pixel location
        """
        i, j, _ = position
        self.map[i, j] = [255, 0, 0]

    def show_robot(self, position, size):
        """Highlights a point in the environment at the given locaiton
        Args:
            position (tuple): pixel location
        """
        _map = self.map.copy()
        image =  cv.circle(_map, (position[1],position[0]), size, (255, 0, 0), -1)
        image = cv.flip(image.astype('uint8'), 0)
        resized = cv.resize(image, (0,0), fx = 2, fy = 2)
        cv.imshow("map", resized)
        return image

    #primitives to save video of the jplanning environment-----------------------
    def begin_video_writer(self):
         self.writer= cv.VideoWriter('Animation Video.mp4', cv.VideoWriter_fourcc(*'DIVX'), 10, (self.width, self.height))
    

    def insert_video_frame(self, image):
        self.writer.write(image)

    def write_video_frame(self):
        image = cv.flip(self.map.astype('uint8'), 0)
        # image = cv.resize(image, (0,0), fx = 2, fy = 2)
        self.writer.write(image)

    def close_video_writer(self):
        self.writer.release()
    #--------------------------------------------------------------------------------
    

if __name__ == "__main__":
    _map_viz = environment(200, 600, 5)
    _map_viz.create_map()
    _map_viz.refresh_map()
    cv.waitKey(0)