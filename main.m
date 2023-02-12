%% parameters description
% Camera Intrinsics and Extrinsics are given specifically in CalibParams.mat and has the following parameters:
% K: has the camera intrinsics 
% (assume that the distortion coefficients in the radtan model are zero).
% TagSize: is in size of each AprilTag in meters.
% mapping.mat
% LeftImgs: is a cell array where each cell is a Image. (reading frames)
% TLeftImgs: is the Timestamps for LeftImgs. For e.g. LeftImgs{1} was collected at time TLeftImgs(1).
% DetAll: is a cell array with AprilTag detections per frame. For e.g., frame 1 detections can be extracted as DetAll{1}. Each cell has multiple rows of data. Each row has the following data format:
% [TagID, p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y]
% Here p1 is the left bottom corner and points are incremented in counter-clockwise direction, i.e., the p1x, p1y are coordinates of the bottom left, p2 is bottom right, p3 is top right, and p4 is top left corners (Refer to Fig. 2). You will use the left bottom corner (p1) of Tag 10 as the world frame origin with positive X being direction pointing from p1 to p2 in Tag 10 and positive Y being pointing from p1 to p4 in Tag 10 and Z axis being pointing out of the plane (upwards) from the Tag.


%% ASSUMPTIONS
% The carpet of april tags are placed over the floor which can be assumed to be a flat surface/plane.
% The quadrotor is moving reasonably slowly.
% Let us choose the first tag detection in the first frame as the world origin and the vector in the direction of p1 to p2 be world X and vector in direction of p1 to p4 be world Y. Refer to figure below.
% All april tags are of the same size and this size is known.
% The april tags are rigidly attached to the ground with minimal effect from the downforce of the propellers of the quadrotor.

%%
clc
clear all
close all

%% Add ToolBox to Path
ToolboxPath = 'gtsam_toolbox';
addpath(ToolboxPath);

%% Data path
Data_path = 'Data';
% choose one from this
% DataMapping
% DataSquare
Data_world_path =  [Data_path, filesep,'DataMapping']; 
path_LeftImgs   =  [Data_world_path, filesep,'frames']; 

%% Load Data
% Download data from the following link: 
% https://drive.google.com/open?id=1ZFXZEv4yWgaVDE1JD6-oYL2KQDypnEUU
load([Data_path, filesep,'CalibParams.mat']);
load([Data_world_path, filesep,'mapping.mat']);
% load frames (T=steps)
% LeftImgs = read_LeftImgs(path_LeftImgs);

%% Initializations
LandMarksComputed = [];
AllPosesComputed = [];
intrinsics = cameraParameters('IntrinsicMatrix',K');
[initial_pose, R, T] = estimate_pose(K, DetAll, TagSize, 1, LandMarksComputed);
AllPosesComputed(1,:) = initial_pose;
Data.R{1} = R;
Data.T{1} = T;
LandMarksComputed = record_world_landmarks_frame(DetAll, Data, intrinsics, 1, LandMarksComputed);

%% Iterate through frames
for frame_no=2:length(DetAll)
	[x, R, T] = estimate_pose(K, DetAll, TagSize, frame_no, LandMarksComputed);
	AllPosesComputed(frame_no,:) = x;
	Data.R{frame_no} = R;
	Data.T{frame_no} = T;
	
	LandMarksComputed = record_world_landmarks_frame(DetAll, Data, intrinsics, frame_no, LandMarksComputed);
end
LandMarksComputed = sortrows(LandMarksComputed,1);



% Z is always positive in our case, so we take the abs value in function
AllPosesComputed(:,3) = abs(AllPosesComputed(:,3));

%% Plot (PRE-GTSAM)
plot_Without_GTSAM(AllPosesComputed, LandMarksComputed);


%% SLAM Using GTSAM (DetAll: is a cell array with AprilTag detections per frame along with the TagID, ...
%  K: Camera Calibration Parameters, TagSize: AprilTag Size in the real world, ...
%  LeftImgs: cell array where each cell is a Image, TLeftImgs: Timestamps for LeftImgs)
[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, Data, AllPosesComputed, LandMarksComputed);
