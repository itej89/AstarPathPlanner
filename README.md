# Implementation of A* Path Planning Algorithm for differential drive action set

## Instructions:
- Clone the repository to your local machine using the following command:
    ```
    cd AstarPathPlanner
    ```

- Alternatively, you can download the zip file of the repository and extract it to your local machine:
    ```
    cd AstarPathPlanner-master
    ```

- Open Terminal and Run the following commands 
    ```
    python3 Astar.py
    ```


## Obstacle Space - Map:

- Result:![Solution](https://user-images.githubusercontent.com/37236721/229381939-12729b64-d03c-4e50-8715-5100f6b28c4a.png)

   

## Demo Video:
 - Video shows exploration of nodes and finding the goal node for a circular robot given start node, goal node, robot radius and clerance from obstacles. Once the shortest path is found, it is backtracked and visualized using openCV.
   
https://user-images.githubusercontent.com/37236721/229382039-71937097-ef83-4d4d-9e34-814406005668.mp4


## Dependencies:
    Install dependancies using "requirements.txt"

    ``` 
    pip install -r requirements.txt
    ```
