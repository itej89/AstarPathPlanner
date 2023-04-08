from a_star import *


class planner:

    wheel_radius = 3.3
    robot_radius = 10.5
    wheel_base_width = 16.0

    goal_dist_threshold = 0
    obstacle_inflation = 5
    step_size = 1


    def __init__(self) -> None:


        print("Enter RPM1 : ") 
        self.RPM1 = int(input())

        print("Enter RPM2 : ") 
        self.RPM2 = int(input())


        self.omega1 = (self.RPM1*2*math.pi)/60
        self.omega2 = (self.RPM2 *2*math.pi)/60

        print("Enter obstacle inflation size : ") 
        self.obstacle_inflation = int(input())

        #Create environment
        print("Please wait while creating the environment.")
        #Create environment for showing the final visualization--------------------------
        #This does not show the additional inflation for robot radius. Insted, it will inflate the robot size
        self.display_environment = environment(200, 600, self.obstacle_inflation, self.step_size, self.wheel_radius, self.wheel_base_width)
        self.display_environment.create_map()
        #-------------------------------------------------------------------------------

        self._environment = environment(200, 600, self.robot_radius+self.obstacle_inflation, self.step_size, self.wheel_radius, self.wheel_base_width)
        self._environment.create_map()
        print("Environment created successfully.")

        # #Getting start node from the user:
        print("\nEnter Start Node as integers: x,y,theta") 
        start_y, start_x, start_theta = input().replace(" ", "").split(',')
        self.start_state = (int(start_x.strip()), int(start_y.strip()), int(start_theta.strip()))

        
        while(not self._environment.is_valid_position(self.start_state)) or self.start_state[2] % 30 != 0: #check if the start node is outside the map
            print("\nStart Node is out of the Map or theta is not a multiple of 30. Re-Enter the Start node: ")
            start_y, start_x, start_theta = input().replace(" ", "").split(',')
            self.start_state = (int(start_x.strip()), int(start_y.strip()), int(start_theta.strip()))
        
        
        # Getting goal node from the user:
        print("Enter Goal Node as integers: x,y,theta")
        goal_y, goal_x, goal_theta = input().replace(" ", "").split(',')
        self.goal_state = (int(goal_x.strip()), int(goal_y.strip()), int(goal_theta.strip()))

        while(not self._environment.is_valid_position(self.goal_state)) or self.goal_state[2] % 30 != 0: #check if the goal node is outside the map
            print("\nGoal Node is out of the Map or theta is not a multiple of 30 Re-Enter the Goal node: ")
            goal_y, goal_x,goal_theta = input().replace(" ", "").split(',')
            self.goal_state = (int(goal_x.strip()), int(goal_y.strip()), int(goal_theta.strip()))

    def plan(self):

        
        self.robot_radius = round(self.robot_radius)
        if self.robot_radius < 1:
            self.robot_radius = 1

        self.goal_dist_threshold = self.robot_radius



         #Create action handler for the environment
        _actionHandler = action_handler(self._environment, self.step_size, self.start_state, self.goal_state, self.omega1, self.omega2, self.wheel_radius, self.wheel_base_width)

        #create planner
        self._astar_planner = astar_planner(self._environment, _actionHandler)

        #Request planner to find a plan
        print("Planning started.")
        print(f"Start Position : {[self.start_state[1], self.start_state[0]]}, Goal Postion : {[self.goal_state[1], self.goal_state[0]]}")
        print("Please wait while searching for the goal state...")
        start_time = time.time()
        _status =  self._astar_planner.find_goal_node(self.start_state, self.goal_state, self.robot_radius, self.goal_dist_threshold, self.display_environment)
        elspsed_time = time.time() - start_time

        print(f"Total time taken to run the algorithm : {elspsed_time} seconds\n")
        
        #If Planner is successfull, visualize the plan
        return _status
        
    def get_trajectory(self):
          return self._astar_planner.back_track()
    
    def get_action_set(self):
          return self._astar_planner.back_track_actions()
    
    def view_plan(self):
        trajectory = self.get_trajectory()
        if trajectory != None:
            # display_environment.begin_video_writer()
            self._astar_planner.visualize_exploration(self.start_state, self.goal_state, self.display_environment, self.robot_radius)
            self._astar_planner.visualize_trajectory(trajectory, self.display_environment, self.robot_radius)
            self._astar_planner.animate_trajectory(trajectory, self.display_environment, self.robot_radius)
            # display_environment.close_video_writer()

            cv.waitKey(0)
    



if __name__ == "__main__":


    _planner = planner()
    if _planner.plan():
        _planner.view_plan()