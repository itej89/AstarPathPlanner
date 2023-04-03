import numpy as np
import cv2
from statistics import mean
import math

#Map Creation using Half Plane Method & OpenCV
class visualization():
    def __init__(self):       

        # Normal Obstacle Space: (points taken in clock-wise order)
        self.rect1_n = [(150,200), (150,75), (165,75), (165,200), (150,200)] #Rect 1 in the given map
        self.rect2_n = [(250,125), (250,0), (265,0), (265,125), (250,125)] #Rect 2 in the given map
        self.cir_n = [(400,110), 50]
        self.wall_n = [(0, 0), (600,0), (600,200), (0,200), (0,0)] #Boundary of the map

        #Empty Lists to store the equations of the lines
        self.rect1_n_eq = []
        self.rect2_n_eq = []
        self.cir_n_eq = []
        self.wall_n_eq = [] #Wall in given map
        
        self.rect1_b_eq = []
        self.rect2_b_eq = []
        self.cir_b_eq = []
        self.wall_b_eq = []

    #Function to calculate the equation of a line given two points
    # ax+by+c=0
    def line_eq(self, pt1, pt2):
        a = pt2[1] - pt1[1] #a = y2 - y1
        b = pt1[0] - pt2[0] #b = x1 - x2
        c = a*(pt1[0]) + b*(pt1[1]) #-c = ax1 + by1
        return [a, b, -c] #return the equation of the line
    
    def check_poly(self, poly, x, y): #Function to check if a node is inside any polygon obstacle space
            count = 0
            if len(poly) > 2:
                for i in range(len(poly)): 
                    if poly[i][0] * x + poly[i][1] * y + poly[i][2] <= 0: #Check if the node is on the left side of the line
                        count+=1 #If yes, increment the count
                
                if count == len(poly): #If the count is equal to the number of lines, the node is inside the polygon
                    return True
                else:
                    return False
                
            else:
                for i in range(len(poly)-1): 
                    if ((x - poly[i][0]) ** 2 + (y - poly[i][1]) ** 2 - poly[i+1] ** 2) <= 0:
                        return True
                    else:
                        return False
            
    # Find the centroid a shape        
    def centroid(self, shape):
        x_coor = [x for x, y in shape]
        y_coor = [y for x, y in shape]
        centroid_x = int(mean(x_coor))
        centroid_y = int(mean(y_coor))

        return (centroid_x, centroid_y)
    
    # Find the bloated line equations of a given shape
    def bloat_eqns(self, shape, clearance):
        shape_b_eq = []        
        if len(shape) > 2:
            centroid_x, centroid_y = self.centroid(shape)
            for i in range(len(shape)-1): #Find the equation of all the lines of obstacle      
                # bloating line equations              
                op = []
                op1 = []            

                op = self.line_eq(shape[i], shape[i+1])
                #print("For points ","(",rect1_n[i], rect1_n[i+1], ")","NOrmal obstacle eqn: ", op[0],"x"," +", op[1],"y"," +",op[2])

                op1 = op.copy()   
                new_op = []            

                # find out if the line is towards the center or away from the center
                loc = op1[0]*centroid_x + op1[1]*centroid_y + op1[2]                 

                new_op.append(op1[0]) # append a
                new_op.append(op1[1]) # append b

                if loc < 0: # line is away from center
                    op1[2] = op1[2] - (clearance * math.sqrt((op1[0]**2) + (op1[1]**2)))
                else: # line is towards  center
                    op1[2] = op1[2] + (clearance * math.sqrt((op1[0]**2) + (op1[1]**2)))
                new_op.append(op1[2])  # append c 
                
                shape_b_eq.append(new_op) #Bloated obstacle eqn
        else:
            shape_b_eq = shape.copy()
            shape_b_eq[1] += clearance

        return shape_b_eq   
    
    def normal_eqns(self, shape):
        shape_n_eq = []
        if len(shape) > 2:
            for i in range(len(shape)-1): #Find the equation of all the lines of shape obstacles
                shape_n_eq.append(self.line_eq(shape[i], shape[i+1])) #Normal shape obstacle eqn
        else:
            shape_n_eq = shape.copy()

        return shape_n_eq          

    #Function to check if a point lies inside a shape - Half Plane Method
    #Each shape's points as per map is taken in clockwise direction and given as a list
    def map_half_plane(self, clearance, robot_radius): 
        
        clearance = clearance + robot_radius
        self.image = np.zeros((600, 200, 3), np.uint8) #create a new image with a black background
        
        self.rect1_n_eq = self.normal_eqns(self.rect1_n)
        self.rect1_b_eq = self.bloat_eqns(self.rect1_n, clearance)

        self.rect2_n_eq = self.normal_eqns(self.rect2_n)
        self.rect2_b_eq = self.bloat_eqns(self.rect2_n, clearance)

        self.cir_n_eq = self.normal_eqns(self.cir_n)
        self.cir_b_eq = self.bloat_eqns(self.cir_n, clearance)

        self.wall_n_eq = self.normal_eqns(self.wall_n)
        self.wall_b_eq = self.bloat_eqns(self.wall_n, -clearance)
   
        for i in range(0, 600): #Loop through all the nodes (pixels) in the map
            for j in range(0, 200): 
                if (self.check_poly(self.rect1_b_eq, i, j) == True): #Check if the node is inside the bloated rect 1 obstacle space
                    if (self.check_poly(self.rect1_n_eq, i, j) == True): #Check if the node is inside the normal rect 1 obstacle space
                        self.image[i][j] = [255, 255, 255]
                    else:
                        self.image[i][j] = [255, 191, 0]
                
                elif (self.check_poly(self.rect2_b_eq, i, j) == True): #Check if the node is inside the bloated rect 1 obstacle space
                    if (self.check_poly(self.rect2_n_eq, i, j) == True): #Check if the node is inside the normal rect 1 obstacle space
                        self.image[i][j] = [255, 255, 255]
                    else:
                        self.image[i][j] = [255, 191, 0]
                
                elif (self.check_poly(self.cir_b_eq, i, j) == True): #Check if the node is inside the bloated rect 1 obstacle space
                    if (self.check_poly(self.cir_n_eq, i, j) == True): #Check if the node is inside the normal rect 1 obstacle space
                        self.image[i][j] = [255, 255, 255]
                    else:
                        self.image[i][j] = [255, 191, 0]
                
                if (self.check_poly(self.wall_n_eq, i, j) == True): #Check if the node is inside the normal wall obstacle space
                    if (self.check_poly(self.wall_b_eq, i, j) == False): #Check if the node is not inside the bloated wall obstacle space 
                        self.image[i][j] = [255, 191, 0] 

        return self.image #Return the image with the map
    
    def highLight_node(self, node, color): #Function to highlight a node in the map
        self.image[node[0],node[1]] = color #Change the color of the node to the specified color

    def highLight_position(self, node, color): #Function to highlight the point robot in the map
        plot = self.image.copy()
        cv2.circle(plot, (node[1],node[0]), 5, color, -1) #Draw a circle at the specified node
        return plot 

def main():
    map = visualization() #Create a map object
    image = map.map_half_plane(5, 5) #Create the map

    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE, image) #Rotate the map by 90 degrees
    image = cv2.resize(image, (0,0), fx=2, fy=2) #scale the map to bigger size
    cv2.imshow("Map", image) #Display the map
    cv2.waitKey(0) #Wait for a key press
    cv2.destroyAllWindows() #Destroy all windows

if __name__ == "__main__":
    main()