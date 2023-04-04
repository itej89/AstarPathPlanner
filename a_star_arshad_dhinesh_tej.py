import sys
import time

import heapq

from node import *
from node_manager import *
from environment import *
from action_handler import *
from math import ceil



#Git repo link https://github.com/itej89/AstarPathPlanner

 
class astar_planner:

    def __init__(self, env, action_handler) -> None:
        self.env = env
        self.action_handler = action_handler

    def goal_check(self, node, list_x, list_y, goal_dist_threshold):
        #Eucledian distance threshold = 1.5 * radius
        #Theta threshold = 30
        dist_matrix = np.sqrt((list_y - node[1])**2 + (list_x - node[0])**2) #dist formula = ((y2 - y1)^2 + (x2 - x1)^2)^1/2
        dist_matrix = dist_matrix.reshape(len(list_y), 1) #reshape to (n,1)

        _x = np.argwhere(dist_matrix < goal_dist_threshold) #get index of all elements in dist_matrix that are less than 0.5
        
    
        if len(_x) > 0: 
            return True

    def visited_node_check(self, node, list_x, list_y, list_theta, robot_radius):
        #Eucledian distance threshold = 0.5
        #Theta threshold = 30
   
        dist_matrix = np.sqrt((list_y - node[1])**2 + (list_x - node[0])**2) #dist formula = ((y2 - y1)^2 + (x2 - x1)^2)^1/2
        dist_matrix = dist_matrix.reshape(len(list_y), 1) #reshape to (n,1)

        _x = np.argwhere(dist_matrix < 0.5*robot_radius).ravel() #get index of all elements in dist_matrix that are less than 0.5
        
        theta_threshold_min = node[2] - 10 #theta threshold min = node's theta -30
        theta_threshold_max = node[2] + 10 #theta threshold max = node's theta +30

        if _x.size > 0:
            thetas = list_theta[_x.ravel()]

            range_thetas = np.where(np.logical_and(thetas>=theta_threshold_min, thetas<=theta_threshold_max))
            
            _len = len(range_thetas[0])
            if _len > 0:
                return True, _x[range_thetas[0][0]]

        if len(_x) > 0: #if there are elements in dist_matrix that are less than 0.5
            for i in _x: #loop through all elements in dist_matrix that are less than 0.5
                if list_theta[i] > theta_threshold_min and list_theta[i] < theta_threshold_max: #if theta of node in list is within theta threshold
                    return True, i #Then node is in the given list
            # return True, _x[0][0]
            return False, None #Else node is not in the given list
        else:
             return False, None #Else node is not in the given list
        
    def explore_actions(self, parent_node) -> bool:
        """Function takes an environmet state and runs 
        possible actions for that state and extract 
        possible unique future states and stores them in the que

        Args:
            parent_node (node): node representing a state of the
            environment

        """

        #get the position of the agent location
        # this represents the (x,y) position on the grid
        agents_postion = parent_node.Node_State

        #move the input node into explored list
        self.visited_node_list.append(parent_node.Node_hash)
        self.node_arrow_list[parent_node.Node_hash] = []

        Sim_Pose = None
        Status = False

        #iterate through all possible actions
        for action in self.action_handler.Actions:
            #make a copy of the current agents position

            #Compute agents possible furture position
            Status ,Sim_Pose = self.action_handler.PerformAction(parent_node, action)

            #If the future position is in the bounds of the environemnt
            if Status:
                
                simulated_position , Total_Cost_To_Come, Total_Cost_To_Go = Sim_Pose
                #else of the state has already been explored then ignore the action
                status, idx = self.visited_node_check(simulated_position, node_manager.node_x[self.visited_node_list], \
                                            node_manager.node_y[self.visited_node_list], node_manager.node_theta[self.visited_node_list], self.robot_radius)
                if status:
                    continue
                
                NewNode = None
                #if state has been visited for the first time then create anew node for the state
                status, idx =  self.visited_node_check(simulated_position, node_manager.node_x, node_manager.node_y, node_manager.node_theta, self.robot_radius)
                if not status:
                    #Create a new node
                    NewNode = node_manager.make_node(simulated_position, Total_Cost_To_Come, Total_Cost_To_Go)
                    #Update the parent for the node
                    NewNode.Parent_Node_hash = parent_node.Node_hash
                    NewNode.Action_to_Node = action
                    heapq.heappush(self.pending_state_que, NewNode)
                    self.node_arrow_list[parent_node.Node_hash].append(NewNode.Node_hash)
                
                
                #else if the state is in pending que then verify its current cost
                # and update its cost and parent if necessary
                else:
                    #If node can be reached in less cost
                    if node_manager.global_node_directory[idx].Total_Cost > Total_Cost_To_Come + Total_Cost_To_Go:
                        #update cost
                        node_manager.global_node_directory[idx].cost_to_come = Total_Cost_To_Come
                        node_manager.global_node_directory[idx].cost_to_go = Total_Cost_To_Go
                        node_manager.global_node_directory[idx].Total_Cost = Total_Cost_To_Come + Total_Cost_To_Go
                        #update new parent node
                        node_manager.global_node_directory[idx].Parent_Node_hash = parent_node.Node_hash
                        node_manager.global_node_directory[idx].Node_State = simulated_position

                        NewNode = node_manager.global_node_directory[idx]
                        NewNode.Action_to_Node = action
                        heapq.heapify(self.pending_state_que)

                #Push new node to the pending que for future expoloration
                # if NewNode != None:
                #     #Found an unique state that needs to be pushed in to the que
                #     heapq.heappush(self.pending_state_que, NewNode)

    
    def find_goal_node(self, start_state, goal_state, robot_radius, goal_dist_threshold, display_env) -> bool:
        """Takes start stat and goal state for the environement 
        and computes the tree using _Breadth first search algorithm till the goal state is reached 

        Args:
            start_state (2D numpy array): Start state for the environment
            goal_state (2D numpy array): Goal state for the environment
        """

        #Call this fuction for auto id generation of nodes for a new tree
        node_manager.initialize()

        #initailize states
        self.robot_radius = robot_radius
        self.goal_state = goal_state
        self.initial_state = start_state
        self.goal_dist_threshold = goal_dist_threshold
        self.Final_Node = None
        self.rendered_nodes = 0

        #Initialize search que. This stores the nodes that needs to be explored
        start_node = node_manager.make_node(self.initial_state)
        self.pending_state_que = [start_node]
        heapq.heapify(self.pending_state_que)

        self.visited_node_list = []
        self.node_arrow_list = {}

        #Perform search till a goal state is reached or maximum number of iterations reached
        
        render_freq = 100
        render_freq_counter  = 0
        while True:
            render_freq_counter += 1
            if render_freq_counter % render_freq == 0:
                self.visualize_exploration(display_env, self.robot_radius, self.rendered_nodes)
                self.rendered_nodes =  len(self.visited_node_list)


            if len(self.pending_state_que) > 0:
                #fetch next node to be explored
                next_item = heapq.heappop(self.pending_state_que)
            else:
                next_item = None
                
                
            if next_item!= None:
                next_node = next_item
                # print(next_node.Node_State)
                #Check if the next node is the goal node, then return success
                if self.goal_check(next_node.Node_State, np.array([goal_state[0]]), np.array([goal_state[1]]), self.goal_dist_threshold):
                    self.Final_Node = next_node
                    print("Found the goal state!!!")
                    return True

                self.explore_actions(next_node)
            else:
                print("Unable to find the goal state!!!")
                return False

    def back_track(self):
        """Funciton to find the final trajectory given the final goal node

        Returns:
            list of tuples: list of trajectory points
        """
        if self.Final_Node != None:
            last_node = self.Final_Node
            trajectory = [last_node.Node_State]

            #back track untill the start node has been reached
            while True:
                last_node = node_manager.global_node_directory[last_node.Parent_Node_hash]
                trajectory.append(last_node.Node_State)

                if last_node.Parent_Node_hash == None:
                    break

            trajectory.reverse()
            return trajectory
        
        return None
    
    def visualize_exploration(self, display_environment, robot_radius, render_node_start_index = 0):
        """Visualize exploration of the nodes
        """
        #indicate start and goal nodes with circles5
        display_environment.highlight_state(start_state, robot_radius, (255, 0, 0))
        display_environment.highlight_state(goal_state, robot_radius, (0, 255, 0))
        display_environment.refresh_map()
        cv.waitKey(1)

        #variable to control gui update rate
        update_count = 0
        # Loop through all visited nodes and show on the cv window
        for idx in self.visited_node_list[render_node_start_index:]:
            if node_manager.global_node_directory[idx].Parent_Node_hash != None:
                parent_idx = node_manager.global_node_directory[idx].Parent_Node_hash
                pose = (node_manager.node_x[parent_idx], node_manager.node_y[parent_idx], \
                        node_manager.node_theta[parent_idx])
                display_environment.update_action(pose, \
                                        node_manager.global_node_directory[idx].Action_to_Node, self.env.is_valid_position)
                
                update_count +=1
                #Update gui every 300 iteration
                if update_count == 500:
                    update_count = 0
                    # display_environment.write_video_frame()
                    cv.waitKey(1)



     

    def visualize_trajectory(self, trajectory, display_environment, robot_radius):
        """Funciton to visualize trajectory

        Args:
            trajectory (lsit of tuples): trajectory
        """
        #Loop through all points in the trajectory
        for point in trajectory:
            display_environment.highlight_point(point)
        #upodate map with the trajectory
        # display_environment.refresh_map()
        # for i in range(20):
        #     display_environment.write_video_frame()
        # cv.waitKey(1)



     

    def animate_trajectory(self, trajectory, display_environment, robot_radius):
        """Funciton to visualize trajectory
        Args:
            trajectory (lsit of tuples): trajectory
        """
        #Loop through all points in the trajectory
        frame = None
        for point in trajectory:
            frame = display_environment.show_robot(point, robot_radius)
            # for i in range(2):
            #     display_environment.insert_video_frame(frame)
            cv.waitKey(100)
           

        cv.waitKey(1)

if __name__ == "__main__":

    #Static parameters--------------------------
    rpm1 = 50
    rpm2 = 100
    wheel_radius = 3.3
    robot_radius = 10.5
    wheel_base_width = 16.0
    #-------------------------------------------

    obstacle_inflation = 5
    # goal_dist_threshold = 1.5 * robot_radius
    goal_dist_threshold = 30

    #the amount of time the robot moves per action
    step_size = 1

    start_state = (30, 30, 30)
    goal_state = (230, 419, 30)


    robot_radius = round(robot_radius)
    if robot_radius < 1:
        robot_radius = 1


    # print("Enter obstacle inflation size : ") 
    # obstacle_inflation = int(input())
    # print("Enter robot radius  : ") 
    # robot_radius = int(input())
    # goal_dist_threshold = 1.5 * robot_radius

    
    # print("Note: Enter step  greater than 0.5 * robot radius")
    # print(f"Enter robot's action step size: [{ceil(0.5 * robot_radius)} - 10]")
    # step_size = int(input())
     
    # while step_size < 1 or step_size > 10: #check if the start node is outside the map
    #     print("\nAction step size out of range [1-10]. Re-Enter the action Step size: ")
    #     step_size = int(input())

    #Create environment
    print("Please wait while creating the environment.")
    #Crate environment for showing the final visualization--------------------------
    #This does not show the additional inflation for robot radius. Insted, it will inflate the robot size
    display_environment = environment(250, 600, obstacle_inflation, step_size, wheel_radius, wheel_base_width)
    display_environment.create_map()
    #-------------------------------------------------------------------------------


    # clearance_robot_radius = round(robot_radius)
    # if clearance_robot_radius < 1:
    #     clearance_robot_radius = 1
    _environment = environment(250, 600, robot_radius+obstacle_inflation, step_size, wheel_radius, wheel_base_width)
    _environment.create_map()
    print("Environment created successfully.")

    #     # #Getting start node from the user:
    # print("\nEnter Start Node as integers: x,y,theta") 
    # start_y, start_x, start_theta = input().replace(" ", "").split(',')
    # start_state = (int(start_x.strip()), int(start_y.strip()), int(start_theta.strip()))

    
    while(not _environment.is_valid_position(start_state)) or start_state[2] % 30 != 0: #check if the start node is outside the map
        print("\nStart Node is out of the Map or theta is not a multiple of 30. Re-Enter the Start node: ")
        start_y, start_x, start_theta = input().replace(" ", "").split(',')
        start_state = (int(start_x.strip()), int(start_y.strip()), int(start_theta.strip()))
    
    
    # # Getting goal node from the user:
    # print("Enter Goal Node as integers: x,y,theta")
    # goal_y, goal_x, goal_theta = input().replace(" ", "").split(',')
    # goal_state = (int(goal_x.strip()), int(goal_y.strip()), int(goal_theta.strip()))

    while(not _environment.is_valid_position(goal_state)) or goal_state[2] % 30 != 0: #check if the goal node is outside the map
        print("\nGoal Node is out of the Map or theta is not a multiple of 30 Re-Enter the Goal node: ")
        goal_y, goal_x,goal_theta = input().replace(" ", "").split(',')
        goal_state = (int(goal_x.strip()), int(goal_y.strip()), int(goal_theta.strip()))

    

    #Create action handler for the environment
    _actionHandler = action_handler(_environment, step_size, start_state, goal_state, rpm1, rpm2, wheel_radius, wheel_base_width)

    #create planner
    _astar_planner = astar_planner(_environment, _actionHandler)

    #Request planner to find a plan
    print("Planning started.")
    print(f"Start Position : {[start_state[1], start_state[0]]}, Goal Postion : {[goal_state[1], goal_state[0]]}")
    print("Please wait while searching for the goal state...")
    start_time = time.time()
    _status =  _astar_planner.find_goal_node(start_state, goal_state, robot_radius, goal_dist_threshold, display_environment)
    elspsed_time = time.time() - start_time

    print(f"Total time taken to run the algorithm : {elspsed_time} seconds\n")

    #If Planner is successfull, visualize the plan
    if _status:
        trajectory = _astar_planner.back_track()
        if trajectory != None:
            # display_environment.begin_video_writer()
            _astar_planner.visualize_exploration(display_environment, robot_radius)
            _astar_planner.visualize_trajectory(trajectory, display_environment, robot_radius)
            _astar_planner.animate_trajectory(trajectory, display_environment, robot_radius)
            # display_environment.close_video_writer()

            display_environment.save_image("Solution.png")
            cv.waitKey(0)

