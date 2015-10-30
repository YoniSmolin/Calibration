% 0 - some constants and bureaucracy

    clear all; clc; close all;
    addpath('PlaneFit');
    pathToDepthImages = 'DepthImages';
    numOfCameras = 2;
    
% 1 - Load the chosen depth images

    depthImage1 = fliplr(imread(fullfile(pathToDepthImages, 'Camera1.png')));
    depthImage2 = fliplr(imread(fullfile(pathToDepthImages, 'Camera2.png')));
    
% 2 - Convert the images to point clouds

    pointCloud1 = depthToCloud(depthImage1);
    pointCloud2 = depthToCloud(depthImage2);

    pointCloud1 = pointCloud(pointCloud1);
    pointCloud2 = pointCloud(pointCloud2);   

% 3 - extract the points corresponding to the planar regions

    roi = [ -inf, inf;   % x
            -inf, inf;   % y
              1 ,  2 ];  % z

    upperLeftCorner1 = struct('x', 239, 'y', 199);
    lowerRightCorner1 = struct('x', 343, 'y', 358);
    [columns, rows] = meshgrid( upperLeftCorner1.x : lowerRightCorner1.x, upperLeftCorner1.y : lowerRightCorner1.y);
    plane1 = pointCloud1.select(rows(:), columns(:));
%     plane1 = plane1.select(plane1.findPointsInROI(roi));
    
    upperLeftCorner2 = struct('x', 242, 'y', 202);
    lowerRightCorner2 = struct('x', 380, 'y', 362);
    [columns, rows] = meshgrid( upperLeftCorner2.x : lowerRightCorner2.x, upperLeftCorner2.y : lowerRightCorner2.y);
    plane2 = pointCloud2.select(rows(:), columns(:));
%     plane2 = plane2.select(plane2.findPointsInROI(roi));
    
    showPointCloud(plane1); hold on;
    showPointCloud(plane2);
    
% 4 - perform regresison to obtain the plane parameters

    [n1,V1,p1] = affine_fit(plane1.Location);
    [n2,V2,p2] = affine_fit(plane2.Location);

% 5 - from the plane parameters deduce the relative rotation between the cameras

    angle = acosd(dot(n1,n2));    
    disp(['The angle between the cameras is: ', num2str(angle)]);
    
% 6 - Use a corner of the board to determine the relative translation

    upperRightCorner1 = struct('x', 232, 'y', 180);
    upperRightCorner2 = struct('x', 233, 'y', 184);
    
    xyz1 = pointCloud1.select(upperRightCorner1.y, upperRightCorner1.x);
    xyz2 = pointCloud2.select(upperRightCorner2.y, upperRightCorner2.x);
    
    R = vrrotvec2mat([cross(n1,n2); deg2rad(-angle)]);
    
    translation = xyz2.Location*R - xyz1.Location;    
    disp(['The distance between the planes is: ', num2str(translation), ' [m]']);
    
% 7 - Merge the two point clouds

    tform = affine3d( [horzcat(R, zeros(3,1)) ; [ -translation 1] ] );
    mergedPointCloud = pcmerge(pctransform(pointCloud2, tform), pointCloud1, 0.1);
    figure; showPointCloud(mergedPointCloud);
    xlabel('x'); ylabel('y'); zlabel('z');