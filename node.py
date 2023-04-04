

#Class to define a tree node contents
class node:

    def __init__(self, state, hash, cost_to_come, cost_to_go) -> None:
        """Constructor for node object
        Args:
            state (numpy array): state of the environemnt
        """

        # an unique id is generated for the given state by taking the 
        # numpy array ordered elements in sequence
        self.Node_hash = hash

        #node's parent id
        # contains "None" if the node is top of the tree
        self.Parent_Node_hash = None

        #Contains state that this node represents
        self.Node_State = state

        #Contains cost-to-come to the node
        self.Cost_to_Come = cost_to_come

        #Contains cost-to-Go to the goal
        self.Cost_to_Go = cost_to_go

        #Contains total-cost to the goal
        self.Total_Cost = self.Cost_to_Come + self.Cost_to_Go
        
        #Contains the action taken to reach this node
        self.Action_to_Node = None

    #Funciton to help heapq for comparing two node objects
    def __lt__(self, other):
        return self.Total_Cost < other.Total_Cost