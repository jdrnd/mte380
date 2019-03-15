Here are high level decriptions of each task, and their assumptions.


#OBJECT_DETECTION
Assumes the obstacle course has the following layout

heading = 90 deg
Y
|
5

4

3

2               starting position? - given by Owen
               /
1             /
             /
0	1	2	3	4	5 - X  -------- heading = 0 deg

Locates the occurance of an object by scanning the lidar buffers for a valid edge detection, and then locates the 
coordinate of the object based on whether the robot it travelling linearly, or turning. Precision should not be a 
large issue since objects only need to be identified within a 12" by 12" square.

#MOTOR_CONTROL

#PROCESS_SENSORS

#READ_SENSORS