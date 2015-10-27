% 0 - some constants

    pathToRecordings = '..\networking\Windows\KinectNetwork\KinectNetwork\Recordings';
    numOfCameras = 2;
    kinect = imaq.VideoDevice('kinect',2);

% 1 - Load the chosen depth images

    for i = 1:numOfCameras
        fullPath = fullfile(pathToRecordings, num2str(i), '100.png');
        depthMM{i} = imread(fullPath, 'PNG');
        pointCloud{i} = depthToPointCloud(depthMM{i}, kinect);
    end

% 2 - Convert the images to point clouds

% 3 - extract the points corresponding to the planar regions

% 4 - perform regresison to obtain the plane parameters

% 5 - from the plane parameters deduce the relative rotation and translation between the cameras