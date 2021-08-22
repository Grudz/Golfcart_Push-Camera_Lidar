/*
Anyone that knows AT LEAST something about anything, 
would know that the cooler the ASCII art at the top
of the files, the better and cooler the programmer


------ [MISSLE ROBOT] ------
     ____          ____
    |oooo|        |oooo|
    |oooo| .----. |oooo|
    |Oooo|/\_||_/\|oooO|
    `----' / __ \ `----'
    ,/ |#|/\/__\/\|#| \,
   /  \|#|| |/\| ||#|/  \
  / \_/|_|| |/\| ||_|\_/ \
 |_\/    o\=----=/o    \/_|
 <_>      |=\__/=|      <_>
 <_>      |------|      <_>
 | |   ___|======|___   | |
//\\  / |O|======|O| \  //\\
|  |  | |O+------+O| |  |  |
|\/|  \_+/        \+_/  |\/|
\__/  _|||        |||_  \__/
      | ||        || |
     [==|]        [|==]
     [===]        [===]
      >_<          >_<
     || ||        || ||
     || ||        || ||
     || ||        || ||    
   __|\_/|__    __|\_/|__
  /___n_n___\  /___n_n___\

*/


// ROS and node class header file
#include <ros/ros.h>
#include "camera_vision.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "camera_vision");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  golfcart_push::CameraVision node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
