%%
clc
clear all
close all

%% visualize frame in 3d
%% Data path
Data_path = 'Data';
% choose one from this
% DataMapping
% DataSquare
Data_world_path =  [Data_path, filesep,'DataMapping']; 
path_LeftImgs   =  [Data_world_path, filesep,'frames']; 
load([Data_path, filesep,'CalibParams.mat']);
Data_world_path =  [Data_path, filesep,'DataMapping']; 
I = imread( [Data_world_path,'\frames\1.jpg']);
imshow(I)
intrinsics = cameraParameters('IntrinsicMatrix',K');
I = undistortImage(I,intrinsics,OutputView="same");
[id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,TagSize);
worldPoints = [0 0 0; TagSize/2 0 0; 0 TagSize/2 0; 0 0 TagSize/2];

for i = 1:length(pose)
    % Get image coordinates for axes.
    imagePoints = worldToImage(intrinsics,pose(i).Rotation, ...
                  pose(i).Translation,worldPoints);

    % Draw colored axes.
    I = insertShape(I,"Line",[imagePoints(1,:) imagePoints(2,:); ...
        imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
        "Color",["red","green","blue"],"LineWidth",7);

    % I = insertText(I,loc(1,:,i),id(i),"BoxOpacity",1,"FontSize",25);
end

imshow(I)