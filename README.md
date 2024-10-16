# Logistic coBot (LB) theme for eYRC 2024-25



# Task 1B

To Launch the gazebo-


```sh
ros2 launch eyantra_warehouse task1b.launch.py
```
To launch moveit

```sh
ros2 launch ur_simulation_gazebo spawn_ur5_launch_moveit.launch.py 
```

# Task 1C

### To Launch the gazebo

```sh
ros2 launch eyantra_warehouse task1c.launch.py  
```

### To launch moveit
```sh
ros2 launch ur_simulation_gazebo spawn_ur5_launch_moveit.launch.py 
```

# Task 2a

### To Launch the gazebo

```sh
ros2 launch eyantra_warehouse task2a.launch.py  
```

### To launch moveit
```sh
ros2 launch ur_simulation_gazebo spawn_ur5_launch_moveit.launch.py 
```


# Task 2B

### To Launch the gazebo

```sh
ros2 launch ebot_description ebot_gazebo_task2b_launch.py
```

### To Launch the payload service

```sh
ros2 run ebot_description payload.py
```

### To launch rviz
```sh
ros2 launch ebot_nav2 ebot_bringup_launch.py 
```
