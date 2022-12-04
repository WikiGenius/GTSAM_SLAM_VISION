%% Wrapper for the CMSC426Course final project at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)
% (Originally for CMSC828T)

clc
clear all
close all

%% ASSUMPTIONS
% The carpet of april tags are placed over the floor which can be assumed to be a flat surface/plane.
% The quadrotor is moving reasonably slowly.
% Let us choose the first tag detection in the first frame as the world origin and the vector in the direction of p1 to p2 be world X and vector in direction of p1 to p4 be world Y. Refer to figure below.
% All april tags are of the same size and this size is known.
% The april tags are rigidly attached to the ground with minimal effect from the downforce of the propellers of the quadrotor.

%% Add ToolBox to Path
ToolboxPath = 'gtsam_toolbox';
addpath(ToolboxPath);

%% Data path
Data_path = 'Data';
% choose one from this
% Data1
% Data2
Data_world_path =  [Data_path, filesep,'Data2']; 
path_LeftImgs   =  [Data_world_path, filesep,'frames']; 

%% Load Data
% Download data from the following link: 
% https://drive.google.com/open?id=1ZFXZEv4yWgaVDE1JD6-oYL2KQDypnEUU
load([Data_path, filesep,'CalibParams.mat']);
load([Data_world_path, filesep,'mapping.mat']);
% load frames (T=steps)
LeftImgs = read_LeftImgs(path_LeftImgs);
% loop over step = 1:T
    % readAprilTag for each step
    % [id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);
    % if step == 1
        % world-origin = first detection which we chose as the origin in the first step
        % To estimate x0 given L110,L210,L310,L410, we’ll use the homography equations
        % R0 and T0 which together constitute the value of x0.

%% SLAM Using GTSAM (DetAll: is a cell array with AprilTag detections per frame along with the TagID, ...
%  K: Camera Calibration Parameters, TagSize: AprilTag Size in the real world, ...
%  LeftImgs: cell array where each cell is a Image, TLeftImgs: Timestamps for LeftImgs)
[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, LeftImgs, TLeftImgs);
