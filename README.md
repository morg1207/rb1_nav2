# Navigation RB1


## Sim Robot

### Mapping

Terminal 1
```bash
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=True
```
Terminal 2

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Launch only localization with Rviz
```bash
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml use_sim_time:=True rviz:=True
```
### Launch only PathPlanner 
```bash
ros2 launch path_planner_server path_planner_server.launch.py type_simulation:=sim_robot use_sim_time:=True 
```

### Launch entire navigation
```bash
ros2 launch path_planner_server navigation.launch.py type_simulation:=sim_robot use_sim_time:=True map_file:=warehouse_map_sim.yaml 
```

## Real Robot
### Mapping
Terminal 1
```bash
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=False
```
Terminal 2

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Launch only localization 
```bash
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml use_sim_time:=True
```
### Launch only PathPlanner 

```bash
ros2 launch path_planner_server navigation.launch.py type_simulation:=real_robot use_sim_time:=False 
```
