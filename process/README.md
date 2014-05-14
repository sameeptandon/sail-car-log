README: 

1) Definition of Frames, Conventions, and File Formats

a) Frames and Conventions
GPS Frame
  - The frame is attached to the IMU in the car and is given by the GPS coordinates in WGS84 convention. Hence, this is: Latitude, Longitude, Height, Azimuth/Heading, Roll, and Pitch. 

ENU Frame
  - The frame is attached to the IMU and is given by East-North-Up convention. We commonly use this frame when computing the difference of WGS84 points (i.e. the difference of two GPS points can be characterized by a translation in the East, North, and Up directions). 

IMU Frame
  - A cartesian coordinate frame attached to the IMU, with the axis printed on the IMU. It is aligned so that +X is forward, +Y left and +Z up. This follows the frame format conventions defined by ROS (http://wiki.ros.org/geometry/CoordinateFrameConventions)
  - To convert from ENU to the IMU frame, we follow the conventions given here: http://www.novatel.com/assets/Documents/Manuals/om-20000124.pdf in section 3.4.7

Camera Frame
  - The camera frame is a cartesian coordinate frame attached to the camera. The frame has +Z out the front of the camera lens, +Y downwards, and +X to the right
  - The IMU to Camera transformation is estimated based on measurements

