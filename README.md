# Self Driving Car - Capstone Project 

Final project of the Udacity Self-Driving Car Engineer Nanodegree.  In this project we need to build different ROS nodes to implement the autonomous vehicle systems


## Architecture

There are 3 areas implemented:

- waypoint updater:  Publish a set of waypoints ahead of the vehicle with correct target velocities.  Velocities should be calculated based on maximum targets and traffic lights, adjusting them to road conditions, like stopping if red traffic light is found, etc.
- traffic light detection: Images of cameras from simulator are received in real-time.  Status of the traffic light (red, yellow, green lights) should be detected in order to adjust the velocities on previous point.
- DBW control: Development of the controllers for the thorttle, brake and steering commands in order to follow waypoints and velocities to those provided by below points.

### Traffic Light detection.

Traffic light status detection is based on image processing, using color and images trasformations with different filters in order to check if existing color ranges are in the provided images.
These images are published in the "/traffic_waypoint" topic.

Below is an example to detect one of the colors (yellow color)

[image1]: ./imgs/traffic_light.png "Image"
[image2]: ./imgs/traffic_light_blue.png "Image"
[image3]: ./imgs/traffic_light_detection.png "Image"

This is the original frame imageobtained from the camera

![Example 1][image1]

We blur  (GaussianBlur) the image and trasform it to HSV color space

![Example 2][image2]

Now we create a mask (in this case yellow color ranges) and transform the image to clean filtered shapes and reducing the possible small noises found in the image.

[For more info: Cv2: Morphological Transformations](https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html)).

![Example 3][image3]

Then we count the number of objects, if it's more than 1 then we have found a positive match.
We do this for the green and yellow/red colors.

### Waypoint Updater

In this node we get the waypoints location, status of traffic lights and vehicle location.  Based on this data we update the desired velocities for each of the waypoints.

This velocities will be defined based in if we find a red/yellow traffic light in front or not.  In case we do it then we smoothly decrease the speed of next waypoints until full stop.

### Control - DBW Node.

Here we adjust the vehicle's controls (throttle, steering and brakes).  We have 2 controllers for this.
- Yaw Controller: It provides the steering commands based on target linear and angular velocity (default)
- Twist Controller: 
    - If we need to reduce velocity (negative velocity error), we just set throttle to zero and brake proportional to error.
    - If we need to aument velocity we use the default PID controller provided.


## Usage (in the simulator)

Install all needed software and dependencies as defined by Udacity and download this repository.

- Start simulator
- Start ROS environment
  ```bash
    - cd ros
    - catkin_make
    - source devel/setup.sh
    - roslaunch launch/styx.launch
  ```

