# Adjustment Factor Graph

In the SfM/SLAM pipeline, I can build the Factor Graph for the Adjustment part as follows:

1. Initial Camera Pose Estimation: 
  I can estimate the initial pose of the camera from just one tag (chosen as the origin) using Homography. This constraint is added to the factor graph as a `PriorFactor`.

2. World Coordinate Origin: 
  The world coordinates of the first tag detected is chosen as the origin and this constraint is added to the factor graph as a `PriorFactor`.

3. Relative Pose between Camera Instants: 
  The relative poses between the camera at consecutive times t and t+1 are close, so I use a `BetweenPointFactor` constraint with Identity transformation in the factor graph.

4. Projection of World Coordinates: 
  The projection of the world coordinates onto the image coordinates is fed into the factor graph using a `GenericProjectionFactor`.

5. Known Tag Size: 
  The size of the tags in the world is known and is fixed, so I feed this constraint into the factor graph using a `BetweenFactor` between the world locations of a particular April tag.

Note: 
  Each factor in the factor graph is modeled with a Gaussian noise model, with the flex in the constraints represented by a covariance matrix. The values of these covariance matrices represent the amount of trust in that particular constraint. For example, I enforce the origin constraint tightly by allowing only 1mm of flex to account for measurement errors. However, I allow the constraint between time instants to be a little more relaxed, such as allowing a 3cm flex between frames. The initial values are given by the pose estimated from Homography, etc.
