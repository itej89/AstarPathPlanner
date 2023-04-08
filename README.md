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
    python3 planner
    ```


## User input example

    ![Terminal Output](https://user-images.githubusercontent.com/37236721/230712148-2eed6cdb-9029-4b84-88ca-ca7dc5babd18.png)


## Demo Video:
 - Video shows exploration of nodes and finding the goal node for a circular robot given start node, goal node, robot radius and clerance from obstacles. Once the shortest path is found, it is backtracked and visualized using openCV.
   


https://user-images.githubusercontent.com/37236721/230712153-aa4c8fe0-315f-4862-9a07-46b8b5b3c287.mp4



## Dependencies:
    Install dependancies using "requirements.txt"

    ``` 
    pip install -r requirements.txt
    ```

## Contributions:
    - environment.py has been updated by Arshad Shaik
    - Visited node closeness check was updated by Dhinesh R
