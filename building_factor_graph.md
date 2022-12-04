Now, we can start building the Factor Graph for the Bundle Adjustment part in the SfM/SLAM pipeline.

- The initial pose of the camera can be estimated from just one tag (first detection which we chose as the origin) using Homography. This constraint is added to the factor graph as a PriorFactor.

- The world coordinates of the first tag detected is chosen as the origin and this constraint is added to the factor graph as a PriorFactor as well.

- The relative poses between the camera at consecutive times t and t+1 are very close as the camera is not moving fast. Hence a BetweenPointFactor constraint with Identity transformation can be used in the factor graph.
- The projection of the world coordinates onto the image coordinates is fed into the factor graph using a GenericProjectionFactor.

- The size of the tags in the world is known and is fixed. This constraint is fed into the factor graph using a BetweenFactor between the world locations of a particular april tag.


Note that each factor in the factor graph is modelled with a gaussian noise model, i.e., the flex in the constraints of the graph are represented by a covariance matrix. The values of these covariance matrix represent the amount of trust in that particular constraint. For eg., we chose the origin so we want to enforce that constraint tightly, so the covariance might say that, I allow only 1mm of flex in this constraint to account for some measurement errors. However, the constraint that we didnt move much between time instants can be a little more relaxed and we might say that we allow it to flex by 3cm between frames and so on. The initial values are given by the pose estimated from Homography and so on.