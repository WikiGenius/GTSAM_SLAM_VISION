function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, LeftImgs, TLeftImgs);
	% For Input and Output specifications refer to the project pdf
    % estimate the pose of the camera/quadrotor at every-time instant 
    % whilst localizing the april tags in the “chosen” world reference frame.
	import gtsam.*
	% Refer to Factor Graphs and GTSAM Introduction
	% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
	% and the examples in the library in the GTSAM toolkit. See folder
	% gtsam_toolbox/gtsam_examples
    
    LandMarksComputed=0;
    AllPosesComputed=0;
end