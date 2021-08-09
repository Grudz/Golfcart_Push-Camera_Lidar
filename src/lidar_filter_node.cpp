/*
Anyone that knows AT LEAST something about anything, 
would know that the cooler the ASCII art at the top
of the files, the better and cooler the programmer

[SEE HUMMER BELOW]
                                              ____
                             ______...-----'_'____\.
             _____.....----==---'\|-----''''        \
            /--------'''''  ____ |                   \
           /  __..--- | .-''    \|\                   \-___
          /| |       || |     __ | \           ____..-'    `---._
         //  |       || |    [__]| |__...----''                  `-.__
 _______//|  |       || |______\\| \ == _____         ____..---''''   \
/_______/ |  `-------'|         `\  |==.     ``---.--'   .-\\\\\\\\\| )
|         | [-]       |[-]          | //          | [ ] (  )|||||||||_'|
|         \           |             |// .-------   \_____`.----''  \ ()|
|    _____ \          |         ___ |` /    ____\_   |   (_) |__..-'   |
\   /     \ \  ____..-| -----'''    | /  .-'      `-_|_               _|
|  /  _--'-\ \         \            | | /    ___     \ |  ____:F_P:-''/
| |  /---_    `-.______|__...------'/ //  .-'   `\    \|_/      __/_-/
 \| / .-. \   _ `--..__\___...-----' | |  |  .-.  |   | ____---'/    |
   | /   \ \  \`-_____....-----------'_|  | (   ) |   |     `--'    /
   | | ( )| |  |__\________/__..-'     \  \  `-'  /   |-'\         /
   \ \    / |  |       \_     _/        \  `-.__.'   /    `--.__.-'
    \ `--' /  /          `---'           \_        _/
     \____/__/                             `------'
     
*/


// ROS and node class header file
#include <ros/ros.h>
#include "lidar_filter.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "lidar_filter");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  golfcart_push::LidarFilter node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
