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
tformguess = rigid3d((rotationM)^-1, -relpos);
% rigidtform3d = 0
tform_ndt = pcregisterndt(movingCloud, staticCloud, gridstep, "InitialTransform", tformguess, "MaxIterations", 10000);


% Apply translation and rotation to second point cloud to match scans
ptCloud2 = (ptCloud2 + tform_ndt.Translation) * ((tform_ndt.R)^-1);


% Convert cartesian coordiates to polar coordinates
% After translation, before NDT scan matching algorithm

ptCloud1_pol = zeros(length(ptCloud1), 3);
ptCloud2_pol = zeros(length(ptCloud2), 3);
for i = 1:length(ptCloud1)
    X = ptCloud1(i,1);
    Y = ptCloud1(i,2);
    Z = ptCloud1(i,3);
    [theta, rho, z] = cart2pol(X,Y,Z);
    ptCloud1_pol(i,:) = [theta, rho, z];
    azi_correct = atan2(Y,X);
    ptCloud1_pol(i,1) = azi_correct;
    rho_correct = sqrt(X^2 + Y^2 + Z^2);
    ptCloud1_pol(i,2) = rho_correct;
    elevation = atan(Z/sqrt(X^2 + Y^2));
    ptCloud1_pol(i,3) = elevation;
end


for i = 1:length(ptCloud2)
    X = ptCloud2(i,1);
    Y = ptCloud2(i,2);
    Z = ptCloud2(i,3);
    [theta, rho, z] = cart2pol(X,Y,Z);
    ptCloud2_pol(i,:) = [theta, rho, z];
    azi_correct = atan2(Y,X);
    ptCloud2_pol(i,1) = azi_correct;
    rho_correct = sqrt(X^2 + Y^2 + Z^2);
    ptCloud2_pol(i,2) = rho_correct;
    elevation = atan(Z/sqrt(X^2 + Y^2));
    ptCloud2_pol(i,3) = elevation;
end



% Plot ptCloud1 and translated ptCloud2 using plot3 function
plot3(ptCloud1(:,1),ptCloud1(:,2),ptCloud1(:,3),'.')
hold on
plot3(ptCloud2(:,1),ptCloud2(:,2),ptCloud2(:,3),'.')



%******** VOXEL DISTRIBUTION **************
% Dividing voxels in a spherical grid as opposed to cartesian 

% NOTE: an extra bound of 5 degrees added to fit voxels 2
% These will mostly be empty

top_elev = sensor.ElevationLimits(1, 2) + 5;
bot_elev = sensor.ElevationLimits(1, 1) - 5;
elev_dif = top_elev - bot_elev;

% Degrees of arclength of voxel  
arc_d = 10; %3.6 %36 
% Amount of voxels divided around circumference
v_az = ceil(360/arc_d);
% Determine amount of bands per elevation voxel
band_amt = 4;
% Determine amount of voxels through elevation
% NOTE: for VLP-16, 1 band every 2 degrees 
v_elev = ceil(elev_dif/band_amt);

% Define amount below horizontal which elevation reaches
zero_el = 0 - sensor.ElevationLimits(1, 1);

% Taking each point within point cloud and placing into desired voxels 
% Create empty array for all voxels 
% First complete for PTCLOUD 1

Voxels1{v_az, v_elev} = [];

for q = 1:length(ptCloud1)
    azi = rad2deg(ptCloud1_pol(q,1));
    rho = ptCloud1_pol(q,2); % rho not needed with infinite voxel length
    elev = rad2deg(ptCloud1_pol(q,3));

    % Create and indice for the azimuth and elevation 
    % Note: Theta (azimuth) goes from -pi to pi
    % Note: Elevation (elev) goes from specified range
    azVoxelInd = ceil((azi+180)/arc_d);
    elevVoxelInd = ceil((elev+zero_el)/band_amt);
    % Add an error check for zero value 
    if azVoxelInd < 1; azVoxelInd = 1; end
    if elevVoxelInd < 1; elevVoxelInd = 1; end 
    % Check if number is a NAN
    if isnan(azi) | isnan(rho) | isnan(elev)
    else
        % Add row number from point cloud to cell array for a given voxel 
        Voxels1{azVoxelInd, elevVoxelInd} = [Voxels1{azVoxelInd, elevVoxelInd} q];
    end
end


% Repeat distribution for PTCLOUD 2

Voxels2{v_az, v_elev} = [];

for q = 1:length(ptCloud2)
    azi = rad2deg(ptCloud2_pol(q,1));
    rho = ptCloud2_pol(q,2); % rho not needed with infinite voxel length
    elev = rad2deg(ptCloud2_pol(q,3));

    % Create and indice for the azimuth and elevation 
    % Note: Theta (azimuth) goes from -pi to pi
    % Note: Elevation (elev) goes from specified range
    azVoxelInd = ceil((azi+180)/arc_d);
    elevVoxelInd = ceil((elev+zero_el)/band_amt);
    % Add an error check for zero value 
    if azVoxelInd < 1; azVoxelInd = 1; end
    if elevVoxelInd < 1; elevVoxelInd = 1; end 
    % Check if number is a NAN
    if isnan(azi) | isnan(rho) | isnan(elev)
    else
        % Add row number from point cloud to cell array for a given voxel 
        Voxels2{azVoxelInd, elevVoxelInd} = [Voxels2{azVoxelInd, elevVoxelInd} q];
    end
end


% Calculate the mean location of each voxel for PTCLOUD 1
%
%

means1{v_az, v_elev} = [];
for i = 1:v_az
    for j = 1:v_elev
        x_m = ptCloud1(Voxels1{i,j},1);
        y_m = ptCloud1(Voxels1{i,j},2);
        z_m = ptCloud1(Voxels1{i,j},3);
        % Check if number is a NAN
        if isnan(x_m) | isnan(y_m) | isnan(z_m)
        else
            mean_pt = [mean(x_m), mean(y_m), mean(z_m)];
            means1{i,j} = mean_pt;
        end
    end
end

% Calculate the mean location of each voxel for PTCLOUD 2
%
%

means2{v_az, v_elev} = [];
for i = 1:v_az
    for j = 1:v_elev
        x_m = ptCloud2(Voxels2{i,j},1);
        y_m = ptCloud2(Voxels2{i,j},2);
        z_m = ptCloud2(Voxels2{i,j},3);
        % Check if number is a NAN
        if isnan(x_m) | isnan(y_m) | isnan(z_m)
        else
            mean_pt = [mean(x_m), mean(y_m), mean(z_m)];
            means2{i,j} = mean_pt;
        end
    end
end

% Calculate the difference in means for ptCloud1 and ptCloud 2
means_dif{v_az, v_elev} = [];
for i = 1:v_az
    for j = 1:v_elev
        % Specifcy a minimum number of points to recognize a voxel 
        if length(Voxels1{i,j}) >= 100 && length(Voxels2{i,j}) >= 100
            x_dif = means2{i,j}(1,1) - means1{i,j}(1,1);
            y_dif = means2{i,j}(1,2) - means1{i,j}(1,2);
            z_dif = means2{i,j}(1,3) - means1{i,j}(1,3);
            mean_dif_pt = [x_dif, y_dif, z_dif];
            means_dif{i,j} = mean_dif_pt;
        else
            means_dif{i,j} = [0 0 0];
        end
    end
end

% Make a 3D quiver plot to represent 
% NOTES
% x changes by column 
% y changes by row
% z changes by third dim

L = target.Dimensions.Length;
W = target.Dimensions.Width;
H = target.Dimensions.Height;

figure()
vox_number = v_az*v_elev;

% Set a double for location in the x of each error point
X_q = zeros(vox_number, 1);% OUt
num = 1;
for i = 1:v_az
    for j = 1:v_elev
        x_ii = means1{i,j}(1,1);
        X_q(num) = x_ii;
        num = num + 1;
    end
end

X_q2 = zeros(vox_number, 1);
num = 1;
for i = 1:v_az
    for j = 1:v_elev
        x_ii = means2{i,j}(1,1);
        X_q2(num) = x_ii;
        num = num + 1;
    end
end

% % Set a double for location in the y of each error point   
Y_q = zeros(vox_number, 1);
num = 1;
for i = 1:v_az
    for j = 1:v_elev
        y_ii = means1{i,j}(1,2);
        Y_q(num) = y_ii;
        num = num + 1;
    end
end

Y_q2 = zeros(vox_number, 1);
num = 1;
for i = 1:v_az
    for j = 1:v_elev
        y_ii = means2{i,j}(1,2);
        Y_q2(num) = y_ii;
        num = num + 1;
    end
end

% % Set a double for location in the z of each error point   
Z_q = zeros(vox_number, 1);
num = 1;
for i = 1:v_az
    for j = 1:v_elev
        z_ii = means1{i,j}(1,3);
        Z_q(num) = z_ii;
        num = num + 1;
    end
end

Z_q2 = zeros(vox_number, 1);
num = 1;
for i = 1:v_az
    for j = 1:v_elev
        z_ii = means2{i,j}(1,3);
        Z_q2(num) = z_ii;
        num = num + 1;
    end
end

% Set a double for location in the r
% Set as mean of PTCLOUD 1 
% R_q(v_az, v_elev, 100) = 0
% for k = 1:

% % Set a double for vector length in the x
U_q = zeros(vox_number, 1);
num = 1;
for i = 1:v_az
    for j = 1:v_elev
        u_ii = means_dif{i,j}(1,1);
        U_q(num) = u_ii;
        num = num + 1;
    end
end

% Set a double for vector length in the y 
V_q = zeros(vox_number, 1);
num = 1;
for i = 1:v_az
    for j = 1:v_elev
        v_ii = means_dif{i,j}(1,2);
        V_q(num) = v_ii;
        num = num + 1;
    end
end

% % Set a double for vector length in the z
W_q = zeros(vox_number, 1);
num = 1;
for i = 1:v_az
    for j = 1:v_elev
        w_ii = means_dif{i,j}(1,3);
        W_q(num) = w_ii;
        num = num + 1;
    end
end

% FORM QUIVER PLOT INDICES 

quiver3(X_q, Y_q, Z_q, U_q, V_q, W_q)
% axis equal

% hold on
% plot3(ptCloud1(Voxels1{12,5},1), ptCloud1(Voxels1{12,5},2), ptCloud1(Voxels1{12,5},3), '.')
% plot3(ptCloud2(Voxels2{12,5},1), ptCloud2(Voxels2{12,5},2), ptCloud2(Voxels2{12,5},3), '.')
% plot3(cosd(-60)*[2, 10], sind(-60)*[2,10], [0,0])
% plot3(cosd(-70)*[2, 10], sind(-70)*[2,10], [0,0])
% hold on
% plot3(ptCloud1(Voxels1{13,5},1), ptCloud1(Voxels1{13,5},2), ptCloud1(Voxels1{13,5},3), '.')
% plot3(ptCloud2(Voxels2{13,5},1), ptCloud2(Voxels2{13,5},2), ptCloud2(Voxels2{13,5},3), '.')
% hold on
% plot3(ptCloud1(Voxels1{11,5},1), ptCloud1(Voxels1{11,5},2), ptCloud1(Voxels1{11,5},3), '.')
% plot3(ptCloud2(Voxels2{11,5},1), ptCloud2(Voxels2{11,5},2), ptCloud2(Voxels2{11,5},3), '.')



figure()
plot3(X_q, Y_q, Z_q, '.')
hold on 
plot3(X_q2, Y_q2, Z_q2, '.')