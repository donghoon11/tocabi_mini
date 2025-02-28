TOCABI mini teaching device (24-25 Winter Internship)
<br/></br>
If you use current control mode, 
```shell
rosrun dynamixel_publisher current_ctrlmode.py
```
If you use current based position control mode,
```shell
rosrun dynamixel_publisher current_pos_ctrlmode.py
```
* rostopic : "/tocabi/Arm_pose"
```
data: 

[0.5675728917121887, 0.05368932709097862, -0.05215534567832947, 0.09357282519340515, -0.006135923322290182, 0.07669904083013535, -0.0015339808305725455, 0.0782330185174942, 

-0.12885437905788422, 0.02454369328916073, 0.03681553900241852, -0.2761165499687195, -0.016873788088560104, 0.09970875084400177, 0.06902913749217987, 0.013805827125906944, 

-0.012271846644580364, 0.016873788088560104]
```
* order


    L_Shoulder1, L_Shoulder2, L_Shoulder3, L_Arm, L_Elbow, L_Forearm, L_Wrist1, L_Wrist2,

    R_Shoulder1, R_Shoulder2, R_Shoulder3, R_Arm, R_Elbow, R_Forearm, R_Wrist1, R_Wrist2,

    L_Gripper, R_Gripper