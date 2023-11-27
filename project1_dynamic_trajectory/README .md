
# Dynamic backup line 

This is assignment 1 dynamic backup line.
![demo](https://github.com/TongshenH/harbinger_assignment/blob/master/project1_dynamic_trajectory/materials/backup_demo.gif)

## Launch 
Run the below command:

`Python3 backup_traj.py` 

Type your desired steering agnle (deg)

## Method

1. Given the wheelbase (L), tread (w), and inner tire steering angle (δi), the Ackermann steering model and the Pythagorean Theorem are used to determine the radius of the path of the front inner tire and the radius of the paths of the rear tires.
![Ackerman steering model](https://github.com/TongshenH/harbinger_assignment/blob/master/project1_dynamic_trajectory/materials/ackerman_steering.png)
![Dynamic backup trajectory form top down view](https://github.com/TongshenH/harbinger_assignment/blob/master/project1_dynamic_trajectory/materials/top_down_view.svg)

2. Project the trajectory from the vehicle frame to the pixel frame. Coordinate transform needs the camera intrinsic and extrinsic matrix. The angle between the horizontal line and camera direction determines how much trajectory is in the camera blind area. Here the zed2i camera is used.
![Camera 0 degree](https://github.com/TongshenH/harbinger_assignment/blob/master/project1_dynamic_trajectory/materials/backup.png)
![Camera 10 degree](https://github.com/TongshenH/harbinger_assignment/blob/master/project1_dynamic_trajectory/materials/backup_10.png)