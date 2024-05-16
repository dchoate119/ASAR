% Daniel Choate
% Goal: to isolate points to distribute to individual voxels 
% To determine major areas impacted by specified errors 

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
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
% aboveIndx1 = find(ptCloud1(:,3) > 6);
% ptCloud1(groundIndx1, :) = [];
% ptCloud1(aboveIndx1, :) = [];
% groundIndx2 = find(ptCloud2(:,3) < 0);
% aboveIndx2 = find(ptCloud2(:,3) > 6);
% ptCloud2(groundIndx2, :) = [];
% ptCloud2(aboveIndx2, :) = [];

movingCloud = pointCloud(ptCloud2);
staticCloud = pointCloud(ptCloud1);
% Set voxel size for NDT alignment
gridstep = 3; %voxel size
%**** Add an initial guess 
alpha = deg2rad(relangle(1));
rotationM = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
% Guess must be in the form of a rotation matrix and a 1x3 translation 
tformguess = rigid3d((rotationM)^-1, -relpos);
% rigidtform3d = 0
tform_ndt = pcregisterndt(movingCloud, staticCloud, gridstep, "InitialTransform", tformguess, "MaxIterations", 10000);

    
tform_icp = pcregistericp(movingCloud, staticCloud);

% Apply translation and rotation to second point cloud to match scans
ptCloud2 = (ptCloud2 + tform_ndt.Translation) * ((tform_ndt.R)^-1);


plot3(ptCloud1(:,1),ptCloud1(:,2),ptCloud1(:,3),'.')
hold on
plot3(ptCloud2(:,1),ptCloud2(:,2),ptCloud2(:,3),'.')


%******** VOXEL DISTRIBUTION *************

% Establish empty cell arrays for different voxels 

L = target.Dimensions.Length;
W = target.Dimensions.Width;
H = target.Dimensions.Height;

% Determine side length of the voxel
v_side = 1;
volume = L*W*H;
v_number = volume/v_side;

% Add 2 additional voxels to support error of NDT (1 should suffice)
% Potentially just add 1 meter?
L_2 = target.Dimensions.Length + 2*v_side; % + 1
W_2 = target.Dimensions.Width + 2*v_side; % + 1
H_2 = target.Dimensions.Height + 2*v_side; % + 1



% Taking each point within point cloud and placing into desired voxels
% Create empty array for all voxels 
% First complete for PTCLOUD 1
Voxels1{ceil(L_2/v_side),ceil(W_2/v_side),ceil(H_2/v_side)} = [];

for q = 1:length(ptCloud1)
    x = ptCloud1(q,1);
    y = ptCloud1(q,2);
    z = ptCloud1(q,3);
    xVoxelInd = ceil((x + L/2)/v_side);
    yVoxelInd = ceil((y + W/2)/v_side);
    zVoxelInd = ceil((z + H/2)/v_side);
    % Added ERROR CHECK FOR ZERO VALUE
    if xVoxelInd < 1; xVoxelInd = 1; end
    if yVoxelInd < 1; yVoxelInd = 1; end
    if zVoxelInd < 1; zVoxelInd = 1; end
    % Check is a number is NAN
    if isnan(x) | isnan(y) | isnan(z)
    else
    % Add row number from point cloud to cell array for a given voxel
    Voxels1{xVoxelInd, yVoxelInd,zVoxelInd} = [Voxels1{xVoxelInd, yVoxelInd,zVoxelInd} q];
    end
    % if q ==30; keyboard; end
end

% Repeat distribution for PTCLOUD 2
%
%


Voxels2{ceil(L_2/v_side),ceil(W_2/v_side),ceil(H_2/v_side)} = [];


for q = 1:length(ptCloud2)
    x = ptCloud2(q,1);
    y = ptCloud2(q,2);
    z = ptCloud2(q,3);
    xVoxelInd = ceil((x + L/2)/v_side);
    yVoxelInd = ceil((y + W/2)/v_side);
    zVoxelInd = ceil((z + H/2)/v_side);
    % Added ERROR CHECK FOR ZERO VALUE
    if xVoxelInd < 1; xVoxelInd = 1; end
    if yVoxelInd < 1; yVoxelInd = 1; end
    if zVoxelInd < 1; zVoxelInd = 1; end
    % Check is a number is NAN
    if isnan(x) | isnan(y) | isnan(z)
    else
    % Add row number from point cloud to cell array for a given voxel
    Voxels2{xVoxelInd, yVoxelInd,zVoxelInd} = [Voxels2{xVoxelInd, yVoxelInd,zVoxelInd} q];
    end
    % if q ==30; keyboard; end
end




% Calculate the mean location of each point cloud for PTCLOUD 1
%
%
means1{ceil(L_2/v_side),ceil(W_2/v_side),ceil(H_2/v_side)} = [];
for i = 1:ceil(L_2/v_side)
    for j = 1:ceil(W_2/v_side)
        for k = 1:ceil(H_2/v_side)
            x_m = ptCloud1(Voxels1{i,j,k},1);
            y_m = ptCloud1(Voxels1{i,j,k},2);
            z_m = ptCloud1(Voxels1{i,j,k},3);
            % Check if number is a NAN
            if isnan(x_m) | isnan(y_m) | isnan(z_m)
            else
            mean_pt = [mean(x_m), mean(y_m), mean(z_m)];
            means1{i,j,k} = [means1{i,j,k} mean_pt];
            end
        end
    end
end

% Calculate the mean location of each point cloud for PTCLOUD 2
%
%
means2{ceil(L_2/v_side),ceil(W_2/v_side),ceil(H_2/v_side)} = [];
for i = 1:ceil(L_2/v_side)
    for j = 1:ceil(W_2/v_side)
        for k = 1:ceil(H_2/v_side)
            x_m = ptCloud2(Voxels2{i,j,k},1);
            y_m = ptCloud2(Voxels2{i,j,k},2);
            z_m = ptCloud2(Voxels2{i,j,k},3);
            % Check if number is a NAN
            if isnan(x_m) | isnan(y_m) | isnan(z_m)
            else
            mean_pt = [mean(x_m), mean(y_m), mean(z_m)];
            means2{i,j,k} = mean_pt;
            end
        end
    end
end

% Calculate the difference in means for ptCloud1 and ptCloud2
means_dif{ceil(L_2/v_side),ceil(W_2/v_side),ceil(H_2/v_side)} = [];
for i = 1:ceil(L_2/v_side)
    for j = 1:ceil(W_2/v_side)
        for k = 1:ceil(H_2/v_side)
            % Set the minimum number of points to recognize a voxel
            if length(Voxels1{i,j,k}) >= 100 && length(Voxels2{i,j,k}) >= 100
                x_dif = means2{i,j,k}(1,1) - means1{i,j,k}(1,1);
                y_dif = means2{i,j,k}(1,2) - means1{i,j,k}(1,2);
                z_dif = means2{i,j,k}(1,3) - means1{i,j,k}(1,3);
                mean_dif_pt = [x_dif, y_dif, z_dif];
                means_dif{i,j,k} = mean_dif_pt;
            else
                means_dif{i,j,k} = [0 0 0];
            end
        end
    end
end



% Plot voxels and point cloud for visualiation
% NOTE: when calling a voxel, row --> column --> height
a = Voxels1{24,10,4};
figure()
plot3(ptCloud1(a,1),ptCloud1(a,2),ptCloud1(a,3), '.')
xlim([-12.5 12.5])
ylim([-12.5 12.5])
zlim([-5.5 5.5])

% Make a 3D quiver plot to represent
% NOTES
% x changes by column 
% y changes by row 
% z changes by third dim
figure()
vox_number = L_2*W_2*H_2;

% Set a double for location in the X
X_q(L_2,W_2,H_2) = 0;
for i = 1:(ceil(L/v_side) + 2*v_side)
    X_q(:,i,:) = i;
end

% Set a double for location in the Y
Y_q(L_2,W_2,H_2) = 0;
for j = 1:(ceil(W/v_side) + 2*v_side)
    Y_q(j,:,:) = j;
end

% Set a double for location in the Z
Z_q(L_2,W_2,H_2) = 0;
for k = 1:(ceil(H/v_side) + 2*v_side)
    Z_q(:,:,k) = k;
end

% Set a double for the vector length in X
U_q(L_2,W_2,H_2) = 0;
for i = 1:(ceil(L/v_side) + 2*v_side)
    for j = 1:(ceil(W/v_side) + 2*v_side)
        for k = 1:(ceil(H/v_side) + 2*v_side)
            U_q(i,j,k) = means_dif{i,j,k}(1,1);
        end
    end
end

% Set a double for the vector length in Y
V_q(L_2,W_2,H_2) = 0;
for i = 1:(ceil(L/v_side) + 2*v_side)
    for j = 1:(ceil(W/v_side) + 2*v_side)
        for k = 1:(ceil(H/v_side) + 2*v_side)
            V_q(i,j,k) = means_dif{i,j,k}(1,2);
        end
    end
end

% Set a double for the vector length in Z
W_q(L_2,W_2,H_2) = 0;
for i = 1:(ceil(L/v_side) + 2*v_side)
    for j = 1:(ceil(W/v_side) + 2*v_side)
        for k = 1:(ceil(H/v_side) + 2*v_side)
            W_q(i,j,k) = means_dif{i,j,k}(1,3);
        end
    end
end


quiver3(X_q, Y_q, Z_q, U_q, V_q, W_q)
% axis equal
vox_number_cart = vox_number;
pos_x = 1;
pos_y = 2;
pos_z = 3;


X_q_cart = quiver_setup_cart(vox_number_cart, means1, L_2, W_2, H_2, pos_x);
Y_q_cart = quiver_setup_cart(vox_number_cart, means1, L_2, W_2, H_2, pos_y);
Z_q_cart = quiver_setup_cart(vox_number_cart, means1, L_2, W_2, H_2, pos_z);
U_q_cart = quiver_setup_cart(vox_number_cart, means_dif, L_2, W_2, H_2, pos_x);
V_q_cart = quiver_setup_cart(vox_number_cart, means_dif, L_2, W_2, H_2, pos_y);
W_q_cart = quiver_setup_cart(vox_number_cart, means_dif, L_2, W_2, H_2, pos_z);

figure()
quiver3(X_q_cart, Y_q_cart, Z_q_cart, U_q_cart, V_q_cart, W_q_cart)


function quiv_i = quiver_setup_cart(vox_number, means, v_L, v_W, v_H, pos)
    quiv_i = zeros(vox_number, 1);
    num = 1;
    for i = 1:v_L
        for j = 1:v_W
            for k = 1:v_H
                position_ii = means{i,j,k}(1,pos);
                quiv_i(num) = position_ii;
                num = num + 1;
            end
        end
    end
end