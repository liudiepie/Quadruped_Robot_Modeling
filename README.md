# Quadruped_Robot_Modeling  
![](<image/testudog.jpg>)   
This is a side project of robotics club at Maryland.  
We try to create a quadrped robot dog which can climb the stairs in the real world.  
This simulation mainly focuses on Forward and Inverse Kinematics of the robot dog.  
In the project, the robot dog of the legs would move in the certain path(oblique circle).  
Using Inverse Kinematics, plugging the speed of the end frame of the leg can transfer to the position.  
## Installation  
1. Clone this repository  
   ```bash
   git clone <HTTPS or SSH>
   ```  
2. Source the code  
   ```bash
   source devel/setup.bash
   ```  
3. Build the file  
   Under the place cloned the repository  
   ```bash
   catkin_make
   ```  
## Launch the file  
1. Create a terminal and   
   ```bash
   roslaunch testudog testudog_launch.launch
   ```  
2. Create another terminal and   
   ```bash
   rosrun testudog main
   ```  
3. The path estimation  
   Under the testudog/controller/src  
   ```bash
   python3 Circle_calculation.py
   ```  
## Result  
This is the performance video [LINK](https://www.youtube.com/watch?v=UkG_kO7CFe8)  
Applying the speed on the front right leg and back left leg, it makes the robot moves around front left leg(ideal condition).  
The moving path of the leg  
![](<image/path.png>)  
