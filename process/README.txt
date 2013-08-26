README:
To generate the perspective-transformed, subsampled training data, you have
to run the GenerateAllLabels script. Its usage typically looks like:

python GenerateAllLabels.py [path to folder of raw data] \
	[path to output data] [path to lane labels] \
	[perspective transform pickle] [list of files to exclude]

The path to the raw data folder should be the highest level folder 
encapsulating data for a single day, so for example,
/scail/group/deeplearning/driving_data/raw_data/7-16-sacramento.

The folder specified in the output path should have two subfolders already 
created in it, 1/ and 2/, corresponding to camera 1 and camera 2's outputs.

The path to lane labels should just contain the enclosing folder, and the
other arguments should refer to the pickle file and the exclude_list
directly.

A typical invocation looks like:
python GenerateAllLabels.py /scail/group/deeplearning/driving_data/raw_data/7-16-sacramento/ /scail/group/deeplearning/driving_data/nikhilb/distance_with_transforms/train/ /scail/group/deeplearning/driving_data/auto_lane_labels/ /scail/group/deeplearning/driving_data/perspective_transforms.pickle exclude_list

In order to generate the un-transformed, un-subsampled test set, you should
invoke the same command but with the flag '--test' appended at the end.