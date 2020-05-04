# OpenPlanner - Global Planner 

## op_global_planner 

This node generate global path from start point to target point(s) on a map. Planning cost is distance only. the algorithm could be extended to handle other costs such as turning right and left busy lanes in the dynamic map. it supports autoware vector map, and special designed .kml maps.

### Outputs
Global path from start to goal, if multiple goals are set, replanning is automatic when the vehicle reaches the end one goal.


### Options (Parameters)
If Lane change is enabled (parralel lanes are detected automatically). 

If Rviz Goal is enabled, Start/Goal(s) are set from Rviz, and saved to .csv files automatically after turning off the node, if rviz param is disables, start/goal(s) will be loaded from  the saved .csv file at.

If replanning is enabled, when the global planning reches the final goal position, it plans a new global path to the start position. so inifite loop of driving around is possible. 

If smoothing is enabled, the generated path's waypoints will have fixed resolution (fixed distance between them) and the path will be slightly smoothed using gradiant descend optimizer. in this case the path density parameter is used to set the waypoints resolution. 

Velocity source options decides which is the reliable autoware's velocity source to use during planning

### Requirements

1. Roadnetwork map (vector map, kml map, lanelet2 map)

### How to launch

* From a sourced terminal:

`roslaunch op_global_planner op_global_planner.launch`

* From Runtime Manager:

Computing Tab -> Mission Planning -> OpenPlanner - Global Planner  -> op_global_planner

### Subscriptions/Publications


```
Publications: 
 * /lane_waypoints_array [autoware_msgs::LaneArray]
 * /global_waypoints_rviz [visualization_msgs::MarkerArray]
 * /op_destinations_rviz [visualization_msgs::MarkerArray]
 * /vector_map_center_lines_rviz [visualization_msgs::MarkerArray]

Subscriptions: 
 * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
 * /move_base_simple/goal [geometry_msgs::PoseStamped]
 * /current_pose [geometry_msgs::PoseStamped]
 * /current_velocity [geometry_msgs::TwistStamped]
 * /vector_map_info/* 
```

![Demo Movie](https://youtu.be/BS5nLtBsXPE)
