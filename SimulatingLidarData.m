% script for simulating lidar scans of input base stl scene (generated using
%autodesk inventor)
clear all 
close all

%import stl
% Locate file to display scene 
% FileName = 'virtual_scenes/scene2.stl'; %round pillars on one side of road
% FileName = 'virtual_scenes/scene2_thick.stl'; %squared pillars
% FileName = 'virtual_scenes/scene2_squares.stl'; %squared pillars
FileName = 'C:\Users\dchoa\OneDrive\Documents\TUFTS\Research\TestEnvironment.stl'; % Test evironment which Dan made 

% Open and read the stl file of point
OpenFile = stlread(FileName);

% Get vertices, faces, and normals from stl
vertices = OpenFile.Points;
faces = OpenFile.ConnectivityList;

%generate extended object mesh
mesh = extendedObjectMesh(vertices,faces);

%rotate mesh to correct orientation
% mesh = rotate(mesh, [0, 0, 90]); %else
% mesh = rotate(mesh, [180, 0, 90]); %else
mesh = rotate(mesh, [270, 0, 90]); %for room

%init lidar unit
% Set mounting location to zero 
% SensorIndex distinguishes point clouds in a multi sensor system 
SensorIndex = 1;
sensor = monostaticLidarSensor(SensorIndex);
sensor.MountingLocation = [0, 0, 0]; %AHHHHAHHHHAHHHH!!!! Why is this not default zero!!??!??!??!



% Set parameters of virtual lidar unit to match velodyne VLP-16 ***
% Cycles per second 
sensor.UpdateRate = 10;
% Degree above and below sensor
sensor.ElevationLimits =  [-22, 10];  % was [-22, 2]; 22
sensor.RangeAccuracy = 0.02; %0.03; %0.01;
% Resolution around sensor 
sensor.AzimuthResolution = 0.05; %0.35;
% Degree between beams 
sensor.ElevationResolution = 0.4; %0.4;
% sensor.MaxRange = 50;
sensor.HasNoise = false;

% Create a tracking scenario. Add an ego platform and a target platform.
scenario = trackingScenario;

% LOCATION OF SENSOR, sets a platform for the scenario 
% ego = platform(scenario, 'Position', [-9.75, -1, 0]);
% ego = platform(scenario, 'Position', [0, 0, 1.72], 'Orientation', eul2rotm(deg2rad([0.0, 45.0, 0.0]))); %[yaw, pitch, roll]
ego = platform(scenario, 'Position', [0, 0, 0]);

% Modifies position of the scene, position raised 3 m in this case 
% Velocity (20 m/s in this case)
% SECOND SCAN IS TAKEN 1 ROTATION LATER
target = platform(scenario,'Trajectory',kinematicTrajectory('Position',[0 0 0],'Velocity',[10 10 0], 'AngularVelocity', [0., 0., 0.5])); %with rotatation 

%NOTE: to use offcentered Position we need to have ZERO AngularVelocity!!!

target.Mesh = mesh;

%specify bounding box for test env ~~~~~~~~~~~~~~
target.Dimensions.Length = 25; 
target.Dimensions.Width = 25;
target.Dimensions.Height = 11; %12; 
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

show(target.Mesh)

% Obtain the mesh of the target viewed from the ego platform after advancing the scenario one step forward.
% Assuming scenario has an automatic 0.1 second value
advance(scenario);
tgtmeshes = targetMeshes(ego);
% Use the created sensor to generate point clouds from the obtained target mesh.
time = scenario.SimulationTime;
[ptCloud1, config, clusters] = sensor(tgtmeshes, time);
blue_cloud = target.Position;
blue_angle = target.Orientation;

%repeat for 2nd scan
advance(scenario);
tgtmeshes = targetMeshes(ego);
time = scenario.SimulationTime;
[ptCloud2, config, clusters] = sensor(tgtmeshes, time);
orange_cloud = target.Position;
orange_angle = target.Orientation;

% Create a loop for multiple simulations
simulations = 5;
tform_ndt_history = zeros(simulations,3);
NDT_Results = zeros(simulations,3);
q = 5/simulations;
grdstp = .25:q:5; % .202334:q:4.9975
NDT_Error = zeros(simulations,3);
for i = 1:simulations
             

    %***************************
    % Use to initialize NDT ****
    relpos = orange_cloud - blue_cloud;
    relangle = orange_angle; % Known that inital angle is zero
    X_Y_Yaw = [relpos(1), relpos(2), deg2rad(relangle(1))];
    %*******************************
    
    % point.GroundPoints = segmentGroundFromLidarData(ptCloud1, "ElevationAngleDelta",SensorIndex)
    
     

    %register two clouds using NDT
    %make pointCloud objects (ndt func doesn't take in xyz coordinates)
    % groundIndx1 = find(ptCloud1(:,3) < 0);
    % ptCloud1(groundIndx1, :) = [];
    % groundIndx2 = find(ptCloud2(:,3) < 0);
    % ptCloud2(groundIndx2, :) = [];
    movingCloud = pointCloud(ptCloud2);
    staticCloud = pointCloud(ptCloud1);
    gridstep = grdstp(i); %voxel size
    %**** Add an initial guess 
    alpha = deg2rad(relangle(1));
    rotationM = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
    % Guess must be in the form of a rotation matrix and a 1x3 translation 
    tformguess = rigid3d((rotationM)^-1, -relpos);
    % rigidtform3d = 0
    tform_ndt = pcregisterndt(movingCloud, staticCloud, gridstep, "InitialTransform", tformguess, "MaxIterations", 10000);
    
    
    tform_icp = pcregistericp(movingCloud, staticCloud);

    tform_ndt_history(i,:) = tform_ndt.Translation
    NDT_Results(i,:) = [tform_ndt.Translation(1) tform_ndt.Translation(2) -acos(tform_ndt.R(1,1))]
    NDT_Error(i,:) = X_Y_Yaw + NDT_Results(i,:)
end
% staticCloud.ZLimits = [-2 12];

% -------------
% Placing own rotations and translations on pt cloud
theta = -20/180 * pi;    %1.5;
myrotmat = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
my_translation = [0 -2 0];

% Creating third point cloud - matching to first 
% Will use the last simulation of the for loop
ptCloud3 = ptCloud2;
% NDT OR ICP TRANLSATIONS AND ROTATIONS
% ptCloud3 = ptCloud3 + tform_icp.Translation; % Just translation ICP
% ptCloud3 = ptCloud3 + tform_ndt.Translation; % Just translation NDT
ptCloud3 = (ptCloud3 + tform_ndt.Translation)*((tform_ndt.R)^-1); %Trans+Rot ndt
% ptCloud3 = (ptCloud3 + tform_icp.Translation) * (tform_icp.R)^-1 ; %Trans+Rot icp

% Putting own rotation on pt cloud3
% ptCloud3 = (ptCloud3 + my_translation) * (myrotmat)^-1;
% ----------------


% COL = ptCloud2(:,3);
% sorted = COL.argsort();

figure()
hold on
axis equal
plot3(ptCloud1(:,1),ptCloud1(:,2),ptCloud1(:,3),'.')
plot3(ptCloud2(:,1),ptCloud2(:,2),ptCloud2(:,3),'.')
plot3(ptCloud3(:,1),ptCloud3(:,2),ptCloud3(:,3),'.')

%remove all NaNss 
% *** EXPLAIN ***
ptCloud1 = rmmissing(ptCloud1);
ptCloud2 = rmmissing(ptCloud2);
ptCloud3 = rmmissing(ptCloud3);
% disp(tform_icp.R);
% disp(myrotmat);

% Plot error figures
t = [1 2 3 4 5];
figure()
hold on
plot(grdstp, abs(NDT_Error(:,1)));
plot(grdstp, abs(NDT_Error(:,2)));
plot(grdstp, 10 * abs(NDT_Error(:,3)));
legend('X (m)', 'Y (m)', 'Yaw (rad * 10)')
title('Translation Error vs. Gridstep')
xlabel('gridstep')
ylabel('Error')

% figure()
% hold on 
% 
% legend('YAW (rad)')
% xlabel('gridstep')
% ylabel('Error')


