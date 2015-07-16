2015 06 26

Author: Riccardo Monica <rmonica[at]ce.unipr.it>
  RIMLab, Department of Information Engineering, University of Parma, Italy
  http://www.rimlab.ce.unipr.it/

-- INTRODUCTION --
This ROS (Robot Operating System) package was developed to fit a Gaussian Mixture Model (GMM) to a trajectory, through Expectation Maximization (EM) algorithm. The main node, gaussian_mixture_model_node, can also be used alone to fit a GMM to arbitrary data, 

Two additional nodes are provided: gaussian_mixture_model_rviz_converter, for visualization, and gaussian_mixture_model_trajectory_converter, for conversion of a trajectory into the required format.

-- GAUSSIAN_MIXTURE_MODEL_NODE --
This node fits the GMM on arbitrary data. The node operates on a set of samples, i.e. tuples of real numbers. These tuples are organized as a matrix, one sample for each row. The EM algorithm treats the samples as an unordered set. 

The input of the node is a std_msgs/Float32MultiArray message. This message must have the layout of a 2D matrix. The first dimension of the layout (at position 0) has size equal to the number of samples (number of rows). The second dimension of the layout has size equal to the number of elements in each tuple (number of columns). The data field of the message contains the matrix, in row-major order.

The input messages are queued and processed in order by the node. For each message, the node attempts to fit GMMs of increasing number of Gaussians. The GMM with the lowest BIC (Bayesian Information Criterion) is chosen as the best model for the data. The attempts stop when a maximum number of Gaussians is reached or the current BIC is much greater than the lowest BIC.

At the end of the processing, a gaussian_mixture_model/GaussianMixture message is produced.
This custom message contains the fields:
  'gaussians': array of gaussian_mixture_model/Gaussian. Each contains:
    'means': mean vector of the Gaussian
    'covariances': covariance matrix of the Gaussian, serialized as an array
                   (it is row-major, but this is irrelevant because the matrix should be symmetric).
  'weights': array of floats, with the same size of 'gaussians'. Contains the weight (prior) assigned to each Gaussian in the model.
  'bic': the BIC value for the model.

See the 'src/gmm_node.h' file for configuration parameters.

-- RVIZ_CONVERTER --
The gaussian_mixture_model_rviz_converter translates a gaussian_mixture_model/GaussianMixture message into a visualization_msg/MarkerArray. This message can be received by the RViz tool and shown in 3D space.

The MarkerArray message contains a set of ellipsoids. These ellipsoids are distorted and rotated such as to represent an equal-probability-density surface of the Gaussians in the GMM. The probability at which the boundary is drawn can be indirectly set through the 'scale' parameter.

Each Gaussian is displayed in a different color, computed by some sort of rainbow function. 

When this node was written, the 'delete all markers' command did not exist yet. Thus, all the possible previously published markers (with ID up to 999, by default) are deleted when sending the new markers.

See the 'src/gmm_rviz_converter.h' file for configuration parameters.

-- TRAJECTORY_CONVERTER --
The gaussian_mixture_model_trajectory_converter node translates a geometry_msgs/PoseArray into a std_msgs/Float32MultiArray, compatible with the gaussian_mixture_model_node node. This is useful to fit a GMM to a trajectory. Each pose in the PoseArray message is converted into a row in the Float32MultiArray matrix.

The parameter 'output_fields' of this node specifies how each pose is converted into a row. This field contains a sequence of space-separated labels. Each of these labels represents a column of the matrix. The specific label describes how the column is computed from the trajectory.
The following labels are currently defined:
  'index': the integer index of the pose in the PoseArray, starting from 0, converted to floating point.
  'x', 'y', 'z': the coordinates of the pose.
  'dx', 'dy', 'dz': the derivative of the coordinates (speed).
For example, if 'output_fields' is "x y z", then the produced matrix has 3 columns (there are 3 labels) corresponding respectively to the x, y, z coordinates of the pose.

-- CREDITS --
The original EM algorithm was taken from the GMM-GMR library by Sylvain Calinon:
  http://sourceforge.net/projects/gmm-gmr/
  http://www.calinon.ch/sourcecodes.php

The algorithm was rewitten using the Eigen library instead of a custom matrix library.

2015 07 16
