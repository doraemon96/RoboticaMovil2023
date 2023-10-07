# This message defines the matches generated in matching 2d features
# from two different cameras.

#######################################################################
#                          Metadata info                              #
#######################################################################

# Time of image acquisition, camera coordinate frame ID
std_msgs/Header header


#######################################################################
#                           Matches data                              #
#######################################################################

# Vectors of 2d points containing point coordinates:
#  - both vectors should contain the same number of elements
#  - both vectors should have the z component of each pointset to 0
#  - both vectors should be ordered such that two points with the same
#     index correspond to matching points
#  - both vectors should correspond to images of the same size
#     (for different image size matches create your own datatype bro)
#  - if stereo, pts1 corresponds to the left image and pts2
#     corresponds to the right image
float32[] pts1
float32[] pts2
