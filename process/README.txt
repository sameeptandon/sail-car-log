README: 

1) Definition of Frames, Conventions, and File Formats

a) Frames and Conventions
GPS Frame
  - The frame is attached to the IMU in the car and is given by the GPS coordinates in WGS84 convention. Hence, this is: Latitude, Longitude, Height, Azimuth/Heading, Roll, and Pitch. 

ENU Frame
  - The frame is attached to the IMU and is given by East-North-Up convention. We commonly use this frame when computing the difference of WGS84 points (i.e. the difference of two GPS points can be characterized by a translation in the East, North, and Up directions). 

IMU Frame
  - A cartesian coordinate frame attached to the IMU, with the axis printed on the IMU. [ TODO lookup the frame coordinate system and write it here ] 
  - To convert from ENU to the IMU frame, we follow the conventions given here: http://www.novatel.com/assets/Documents/Manuals/om-20000124.pdf in section 3.4.7

Camera Frame
  - The camera frame is a cartesian coordinate frame attached to the camera. The frame has +Z out the front of the camera lens, +Y downwards, and +X to the right
  - The IMU to Camera transformation is estimated based on measurements

b) File Formats
  TODO [talk about OpenDrive, how our lane labels are stored, etc] 


README:
To generate the perspective-transformed, subsampled training data, you have
to run the GenerateAllLabels script. Its usage typically looks like:

python GenerateAllLabels.py [path to folder of raw data] \
	[path to output data] [path to lane labels] \
	[perspective transform pickle] [list of files to exclude]

The path to the raw data folder should be the highest level folder 
encapsulating data for a single day, so for example,
/scail/group/deeplearning/driving_data/raw_data/7-16-sacramento.
This raw data folder has the *_gps.out files and the splits in its recursive
subdirectories.

The folder specified in the output path should have two subfolders already 
created in it, 1/ and 2/, corresponding to camera 1 and camera 2's outputs.

The path to lane labels should be the folder containing those labels. The
labels are outputted by LaneClickTest.py after being piped through
interp_2d.py

The perspective transform pickle file should be specified directly. (It's 
currently at /scail/group/deeplearning/driving_data/auto_lane_labels/ /scail/group/deeplearning/driving_data/perspective_transforms.pickle)

The list of files to exclude should be specified directly. (It's currently
at sail-car-log/process/exclude_list).

A typical invocation looks like:
python GenerateAllLabels.py /scail/group/deeplearning/driving_data/raw_data/7-16-sacramento/ /scail/group/deeplearning/driving_data/nikhilb/distance_with_transforms/train/ /scail/group/deeplearning/driving_data/auto_lane_labels/ /scail/group/deeplearning/driving_data/perspective_transforms.pickle exclude_list

In order to generate the un-transformed, subsampled test set, you should
invoke the same command but with the flag '--test' appended at the end.

To generate the un-transformed, un-subsampled test set, you should find the 
line in TestLabelGenerator.py, containing `video_reader.setSubsample(true)`
and comment it out. You should also change the line containing
`frame = 10*count` and change it to `frame = count`.
