function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, Data, AllPosesComputed, LandMarksComputed);
	% For Input and Output specifications refer to the project pdf
    % estimate the pose of the camera/quadrotor at every-time instant 
    % whilst localizing the april tags in the “chosen” world reference frame.

	%% import libraries
	import gtsam.*
    % https://gtsam.org/tutorials/intro.html
    
	% Refer to Factor Graphs and GTSAM Introduction
	% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
	% and the examples in the library in the GTSAM toolkit. See folder
	% gtsam_toolbox/gtsam_examples

    %% Factor Graph
	graph = NonlinearFactorGraph;
	initialEstimate = Values;
	
	% Add prior for the first pose
	graph.add(PriorFactorPose3(symbol('x',1),Pose3(Rot3(Data.R{1}), Point3(Data.T{1})), noiseModel.Diagonal.Sigmas(ones(6,1) * .001)));
	% Add prior for the World origin
	graph.add(PriorFactorPoint3(symbol('L',10),Point3(0,0,0),noiseModel.Diagonal.Sigmas(ones(3,1) * 1e-6)));
	% Add identity between factor between poses
	for i = 1:size(DetAll,2)-1
		p1 = Pose3(Rot3(Data.R{i}),Point3(Data.T{i}));
		p2 = Pose3(Rot3(Data.R{i+1}),Point3(Data.T{i+1}));      
		id = Pose3(eye(4,4));
		
		graph.add(BetweenFactorPose3(symbol('x',i),symbol('x',i+1), ...
			id, noiseModel.Diagonal.Sigmas(ones(6,1) * .001)));
	end
	
	% Projection factors
	Kp = Cal3_S2(K(1, 1), K(2, 2), 0, K(1,3), K(2, 3));
	for i = 1:size(DetAll,2)
		frame = DetAll{i};
		for j = 1:size(frame,1)
			graph.add(GenericProjectionFactorCal3_S2(...
				Point2(frame(j,2),frame(j,3)), noiseModel.Isotropic.Sigma(2,1.0), symbol('x',i),...
				symbol('L',frame(j,1)), Kp));
			graph.add(GenericProjectionFactorCal3_S2(...
				Point2(frame(j,4),frame(j,5)), noiseModel.Isotropic.Sigma(2,1.0), symbol('x',i),...
				symbol('M',frame(j,1)), Kp));
			graph.add(GenericProjectionFactorCal3_S2(...
				Point2(frame(j,6),frame(j,7)), noiseModel.Isotropic.Sigma(2,1.0), symbol('x',i),...
				symbol('N',frame(j,1)), Kp));
			graph.add(GenericProjectionFactorCal3_S2(...
				Point2(frame(j,8),frame(j,9)), noiseModel.Isotropic.Sigma(2,1.0), symbol('x',i),...
				symbol('O',frame(j,1)), Kp));
		end
	end
	
	% Add between factors between tags
	for i = 1:size(LandMarksComputed,1)
		graph.add(BetweenFactorPoint3(symbol('L',LandMarksComputed(i,1)),...
			symbol('M',LandMarksComputed(i,1)),Point3(TagSize, 0, 0),noiseModel.Diagonal.Sigmas(ones(3,1) * 1e-6)));
		graph.add(BetweenFactorPoint3(symbol('L',LandMarksComputed(i,1)),...
			symbol('O',LandMarksComputed(i,1)),Point3(0, TagSize, 0),noiseModel.Diagonal.Sigmas(ones(3,1) * 1e-6)));
		graph.add(BetweenFactorPoint3(symbol('M',LandMarksComputed(i,1)),...
			symbol('N',LandMarksComputed(i,1)),Point3(TagSize, 0, 0),noiseModel.Diagonal.Sigmas(ones(3,1) * 1e-6)));
		graph.add(BetweenFactorPoint3(symbol('O',LandMarksComputed(i,1)),...
			symbol('N',LandMarksComputed(i,1)),Point3(0, TagSize, 0),noiseModel.Diagonal.Sigmas(ones(3,1) * 1e-6)));
	end
	
	for i = 1:size(DetAll,2)
		initialEstimate.insert(symbol('x',i), Pose3(Rot3(Data.R{i}), Point3(Data.T{i})));
	end
	
	for i = 1:size(LandMarksComputed,1)
	   initialEstimate.insert(symbol('L',LandMarksComputed(i, 1)),Point3([LandMarksComputed(i, 2:3) 0]'));
	   initialEstimate.insert(symbol('M',LandMarksComputed(i, 1)),Point3([LandMarksComputed(i, 4:5) 0]'));
	   initialEstimate.insert(symbol('N',LandMarksComputed(i, 1)),Point3([LandMarksComputed(i, 6:7) 0]'));
	   initialEstimate.insert(symbol('O',LandMarksComputed(i, 1)),Point3([LandMarksComputed(i, 8:9) 0]')); 
	end
	
	LMf = LevenbergMarquardtParams;
	LMf.setlambdaInitial(1.0);
	LMf.setVerbosityLM('trylambda');
	optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, LMf);
	result = optimizer.optimize();
	
	%% Retrieving landmarks
	for i = 1:size(LandMarksComputed,1)
		L(i, :) = [result.at(symbol('L', LandMarksComputed(i, 1))).x result.at(symbol('L', LandMarksComputed(i, 1))).y result.at(symbol('L', LandMarksComputed(i, 1))).z];
		M(i, :) = [result.at(symbol('M', LandMarksComputed(i, 1))).x result.at(symbol('M', LandMarksComputed(i, 1))).y result.at(symbol('M', LandMarksComputed(i, 1))).z];
		N(i, :) = [result.at(symbol('N', LandMarksComputed(i, 1))).x result.at(symbol('N', LandMarksComputed(i, 1))).y result.at(symbol('N', LandMarksComputed(i, 1))).z];
		O(i, :) = [result.at(symbol('O', LandMarksComputed(i, 1))).x result.at(symbol('O', LandMarksComputed(i, 1))).y result.at(symbol('O', LandMarksComputed(i, 1))).z];
	end
	LandMarksComputed = [LandMarksComputed(:, 1) L M N O];
	
	AllPosesComputed(:,3) = abs(AllPosesComputed(:,3));
	
	%% % Plot the last two figures (POST-GTSAM)
	plot_With_GTSAM(AllPosesComputed, LandMarksComputed);
end