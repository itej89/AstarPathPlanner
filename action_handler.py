import math
import time
import numpy as np

class action_handler:
#Class to handle the action supported by the environment


    def __init__(self, env, step_size, start_pose, goal_pose, omega1, omega2, wheel_radius, wheel_base_width) -> None:
        """Initilaize action handler parameters
        Args:
            env (environment): referance ot environment
        """
        self.env = env
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.step_size = step_size
        self.robot_wheel_radius = wheel_radius
        self.wheel_base_width = wheel_base_width

        #define al the actions possible in the environment
        self.Actions = [(0, omega1), (omega1,0), (omega1, omega1), (0, omega2), (omega2, 0), (omega2,omega2), (omega1, omega2), (omega2, omega1)]
        print(f"ACTION SET : {self.Actions}")


    def cost_norm(self, pose1, pose2):
        x1, y1, _ = pose1
        x2, y2, _ = pose2
        return np.linalg.norm([x1 - x2, y1 - y2])

    def ActionValues(self, pose, action):
        Xi,Yi,Thetai = pose
        UL, UR = action

        t = 0
        dt = 0.1

        Xn=Xi
        Yn=Yi
        Thetan = 3.14 * Thetai / 180


        cost_to_go=0
        while t<self.step_size:
            t = t + dt

            Delta_Xn = 0.5*self.robot_wheel_radius * (UL + UR) * math.cos(Thetan) * dt
            Delta_Yn = 0.5*self.robot_wheel_radius * (UL + UR) * math.sin(Thetan) * dt

            Thetan += (self.robot_wheel_radius / self.wheel_base_width) * (UR - UL) * dt

            cost_to_go=cost_to_go+ math.sqrt(math.pow((0.5*self.robot_wheel_radius * (UL + UR) * math.cos(Thetan) *
            dt),2)+math.pow((0.5*self.robot_wheel_radius * (UL + UR) * math.sin(Thetan) * dt),2))

            Xn += Delta_Xn
            Yn += Delta_Yn

            if not self.env.is_valid_position((round(Xn), round(Yn), Thetan)):
                return None, None

        Thetan = 180 * (Thetan) / 3.14
        return (round(Xn), round(Yn), Thetan%360), cost_to_go
    
    def PerformAction(self, parent_node, action):
        """Compute the next state and estimate the total cost for the next state

        Args:
            parent_node (node): node of the 
            action (string): action label

        Returns:
            tuple: validity of the action, action cost
        """

        agents_pose = parent_node.Node_State

        #Simulate agents next position
        simulated_position, action_cost = self.ActionValues(agents_pose, action)
        
        if simulated_position is not None:

            #Compute the total cost of the action
            action_cost_to_come = parent_node.Cost_to_Come + action_cost

            action_cost_to_go =  self.cost_norm(simulated_position, self.goal_pose)#+abs(agents_pose[2] - simulated_position[2])

            #return the estimated position and cost
            return True, (simulated_position, action_cost_to_come, action_cost_to_go)

        else:
            return False, None
            