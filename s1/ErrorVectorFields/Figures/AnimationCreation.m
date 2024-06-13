% Daniel Choate
% Goal: to convert cartesian coordinates to polar coordinates

% script for simulating lidar scans of input base stl scene (generated using
%autodesk inventor)
clear all 
close all

%import stl
% Locate file to display scene 
% Test evironment which Dan made
FileName = 'C:\Users\dchoa\OneDrive\Documents\TUFTS\Research\TestEnvironment.stl';  

% Open and read the stl file of point
OpenFile = stlread(FileName);

% Get vertices, faces, and normals from stl
vertices = OpenFile.Points;
faces = OpenFile.ConnectivityList;

% Generate extended object mesh
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
% LOCATION NOT DEFAULT ZERO 
sensor.MountingLocation = [0, 0, 0];

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
ego = platform(scenario, 'Position', [0, 0, 0]);

% Modifies position of the scene, position raised 3 m in this case 
% Velocity (20 m/s in this case)
% SECOND SCAN IS TAKEN 1 ROTATION LATER
target = platform(scenario,'Trajectory',kinematicTrajectory('Position',[0 0 3],'Velocity',[10 10 0], 'AngularVelocity', [0., 0., 0.5])); %with rotatation 

target.Mesh = mesh;

%specify bounding box for test env ~~~~~~~~~~~~~~
% Specific for the test scene which Dan created 
target.Dimensions.Length = 25; %32
target.Dimensions.Width = 25; %32
target.Dimensions.Height = 11; %16; %12 

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

% NDT_Error = zeros(simulations,3);

relpos = orange_cloud - blue_cloud;
relangle = orange_angle; % Known that inital angle is zero
X_Y_Yaw = [relpos(1), relpos(2), deg2rad(relangle(1))];
% Remove ground plane points 
% groundIndx1 = find(ptCloud1(:,3) < 0);
% % aboveIndx1 = find(ptCloud1(:,3) > 2);
% ptCloud1(groundIndx1, :) = [];
% % ptCloud1(aboveIndx1, :) = [];
% groundIndx2 = find(ptCloud2(:,3) < 0);
% % aboveIndx2 = find(ptCloud2(:,3) > 2);
% ptCloud2(groundIndx2, :) = [];
% % ptCloud2(aboveIndx2, :) = [];


movingCloud = pointCloud(ptCloud2); % POLAR CLOUD
staticCloud = pointCloud(ptCloud1); % POLAR CLOUD

% Set voxel size for NDT alignment
gridstep = 3; %voxel size
%**** Add an initial guess 
alpha = deg2rad(relangle(1));
rotationM = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
% Guess must be in the form of a rotation matrix and a 1x3 translation 
% tformguess = rigid3d((rotationM)^-1, -relpos);
% rigidtform3d = 0
tform_ndt = pcregisterndt(movingCloud, staticCloud, gridstep, "MaxIterations", 100); % "InitialTransform", tformguess,


% Apply translation and rotation to second point cloud to match scans
ptCloud2 = (ptCloud2 + tform_ndt.Translation) * ((tform_ndt.R)^-1);

% Plot ptCloud1 and translated ptCloud2 using plot3 function
plot3(ptCloud1(:,1),ptCloud1(:,2),ptCloud1(:,3),'.')
hold on
plot3(ptCloud2(:,1),ptCloud2(:,2),ptCloud2(:,3),'.')
