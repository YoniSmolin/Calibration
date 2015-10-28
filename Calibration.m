% 0 - some constants and bureaucracy

    clear all; clc; close all;
    addpath('PlaneFit');
    pathToDepthImages = 'DepthImages';
    numOfCameras = 2;
    
% 1 - Load the chosen depth images

    depthImage1 = imread(fullfile(pathToDepthImages, 'Camera1.png'));
    depthImage2 = imread(fullfile(pathToDepthImages, 'Camera2.png'));
    
% 2 - Convert the images to point clouds

    pointCloud1 = depthToCloud(depthImage1);
    pointCloud2 = depthToCloud(depthImage2);

    pointCloud1 = pointCloud(pointCloud1);
    pointCloud2 = pointCloud(pointCloud2);   

% 3 - extract the points corresponding to the planar regions

    roi = [ -inf, inf;   % x
            -inf, inf;   % y
              1 ,  2 ];  % z

    upperLeftCorner1 = struct('x', 170, 'y', 183);
    lowerRightCorner1 = struct('x', 277, 'y', 347);
    [rows, columns] = meshgrid( upperLeftCorner1.x : lowerRightCorner1.x, upperLeftCorner1.y : lowerRightCorner1.y);
    plane1 = pointCloud1.select(rows(:), columns(:));
    plane1 = plane1.select(plane1.findPointsInROI(roi));
    
    upperLeftCorner2 = struct('x', 139, 'y', 212);
    lowerRightCorner2 = struct('x', 257, 'y', 367);
    [rows, columns] = meshgrid( upperLeftCorner2.x : lowerRightCorner2.x, upperLeftCorner2.y : lowerRightCorner2.y);
    plane2 = pointCloud2.select(rows(:), columns(:));
    plane2 = plane2.select(plane2.findPointsInROI(roi));
    
    showPointCloud(plane1); hold on;
    showPointCloud(plane2);
% 4 - perform regresison to obtain the plane parameters

    [n1,V1,p1] = affine_fit(plane1.Location);
    [n2,V2,p2] = affine_fit(plane2.Location);

% 5 - from the plane parameters deduce the relative rotation and translation between the cameras

    angle = acosd(dot(n1,n2));
    angle = min(angle, 180 - angle);
    disp(['The angle between the planes is: ', num2str(angle)]);