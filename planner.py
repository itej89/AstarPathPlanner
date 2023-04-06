from a_star import *

class planner:

    wheel_radius = 3.3
    robot_radius = 10.5
    wheel_base_width = 16.0

    goal_dist_threshold = 0
    obstacle_inflation = 5
    step_size = 1
    def plan(self, rpm1, rpm2, start_state, goal_state):
        
        self.robot_radius = round(self.robot_radius)
        if self.robot_radius < 1:
            self.robot_radius = 1

        self.goal_dist_threshold = self.robot_radius
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


         #Create action handler for the environment
        _actionHandler = action_handler(self._environment, self.step_size, start_state, goal_state, rpm1, rpm2, self.wheel_radius, self.wheel_base_width)

        #create planner
        self._astar_planner = astar_planner(self._environment, _actionHandler)

        #Request planner to find a plan
        print("Planning started.")
        print(f"Start Position : {[start_state[1], start_state[0]]}, Goal Postion : {[goal_state[1], goal_state[0]]}")
        print("Please wait while searching for the goal state...")
        start_time = time.time()
        _status =  self._astar_planner.find_goal_node(start_state, goal_state, self.robot_radius, self.goal_dist_threshold, self.display_environment)
        elspsed_time = time.time() - start_time

        print(f"Total time taken to run the algorithm : {elspsed_time} seconds\n")
        
        #If Planner is successfull, visualize the plan
        return _status
        
    def get_trajectory(self):
          return self._astar_planner.back_track()
    
    def get_action_set(self):
          return self._astar_planner.back_track_actions()
    
    def view_plan(self, start_state, goal_state, trajectory):
        if trajectory != None:
            # display_environment.begin_video_writer()
            self._astar_planner.visualize_exploration(start_state, goal_state, self.display_environment, self.robot_radius)
            self._astar_planner.visualize_trajectory(trajectory, self.display_environment, self.robot_radius)
            self._astar_planner.animate_trajectory(trajectory, self.display_environment, self.robot_radius)
            # display_environment.close_video_writer()

            cv.waitKey(0)
    




if __name__ == "__main__":
    rpm1 = 50
    rpm2 = 100

    start_state = (30, 30, 30)
    goal_state = (100, 500, 30)

    _planner = planner()
    if _planner.plan(rpm1, rpm2, start_state, goal_state):
        _planner.view_plan(start_state, goal_state, _planner.get_trajectory())
        print(_planner.get_action_set())