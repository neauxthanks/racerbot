## General info
This is PA6 - Line Follow.

## To start
1 roslaunch aldopkg linemission.launch model:=waffle #specifies which model
2 rosrun aldopkg lightning_mcqueen.py 

## Description
It spawns a waffle robot at the start of a yellow line from linemission. The robot follows the line as closely as possible and keeps going as indicated from the line. Once it runs out of line, however, it will continue following that direction. Three windows pop up that show the relevant masks along with the blue line and centeroid showing the direction of the robot and what it is following.
 
## Explanation
lightning_mcqueen basically filters out the unnecessary visual information of space by creating masks in order to calculate the center of the yellow line using the centroid as representation of the following algorithm. With this, the robot is able to follow the line as it goes.

## Video
you can find the video in the assignmentstuff folder!