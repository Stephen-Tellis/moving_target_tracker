# Real-time target position tracking

Enables switching between trajectory execution and real-time servo control to track a moving target. 

![](assets/demo.gif)   

`point1` and `point2` move at different speeds and are at different 3D locations.

## What's under the hood?
moveit_servo  
KDL engine (LMA IK solver)    
pilz_industrial_motion (global planners work too)  

