% Daniel Choate 
% Goal: Use quiver plot analyses for different data sets 
% Main code for paper 

% script for simulating lidar scans of input base stl scene (generated using
%autodesk inventor)
clear all 
close all

%import stl
% Locate file to display scene 
% Test evironment which Dan made
FileName = 'C:\Users\dchoa\OneDrive\Documents\TUFTS\Research\Solidworks\TestEnvironment.stl';  


% % ********************** UPLOADING VOLPE DATA ***************************
% cd 'C:\Users\dchoa\Box\GPS-INS Project Materials\Data-Volpe\Lidar_RelativePosition'
% flagReadData = true; %false;
% flagPlayMovie = true;
% % Read VLP-16 Data
% if flagReadData
%     veloReader = velodyneFileReader('2021-03-10-16-43-50_Velodyne-VLP-16-Data_garminSignage.pcap','VLP16')
% end
% 
% % ***********************************************************************

% ********************** IMPORT VOLPE DATA *******************************

% Syntax to read SignagePCs.mat
% cd 'C:\Users\dchoa\Box\GPS-INS Project Materials\Data-Volpe\Lidar_RelativePosition'
load signagePCs
size(pc);

% Extract first point cloud 
im1=pc{1}; 
im2=pc{2};
im3=pc{3};
im4=pc{4};
im5=pc{5};
im6=pc{6};
im7=pc{7};
im8=pc{8};
im9=pc{9};
im10=pc{10};

ptCloud1 = create_cloud(im1);
ptCloud2 = create_cloud(im2);
ptCloud3 = create_cloud(im3);
ptCloud4 = create_cloud(im4);
ptCloud5 = create_cloud(im5);
ptCloud6 = create_cloud(im6);
ptCloud7 = create_cloud(im7);
ptCloud8 = create_cloud(im8);
ptCloud9 = create_cloud(im9);
ptCloud10 = create_cloud(im10);


% % Plot pt clouds 1 and 10 to visualize largest change
% figure()
% plot3(ptCloud1(:,1), ptCloud1(:,2), ptCloud1(:,3), '.')
% hold on 
% plot3(ptCloud10(:,1), ptCloud10(:,2), ptCloud10(:,3), '.')

movingCloud = pointCloud(ptCloud10); 
staticCloud = pointCloud(ptCloud1); 

% First NDT scan matching test 
% Set voxel size for NDT alignment
gridstep = 0.125; %voxel size
%**** Add an initial guess 
alpha = 0;
rotationM = [cos(alpha) sin(alpha) 0; -sin(alpha) cos(alpha) 0; 0 0 1];
pos_guess = [0 -3 0];
% Guess must be in the form of a rotation matrix and a 1x3 translation 
tformguess = rigid3d((rotationM)^-1, pos_guess);
% rigidtform3d = 0
tform_ndt = pcregisterndt(movingCloud, staticCloud, gridstep, "InitialTransform", tformguess) %,"MaxIterations", 10000); 

% Calculate error of the first NDT scan match 
% WITHOUT SHADOW MITIGATION
tform_ndt_tran = tform_ndt.Translation;
% NDT_Results = [tform_ndt.Translation(1) tform_ndt.Translation(2) -acos(tform_ndt.R(1,1))];


% All shadowing and voxel distribution done after alignment
% **************** ALIGNING SCANS USING NDT ***************
ptCloud10b = ptCloud10;
ptCloud10 = (ptCloud10 - tform_ndt.Translation);% * ((tform_ndt.R)^-1);

% *************** ALIGNING SCANS USING TRUTH **************
% ADD TRUTH ONCE CALCULATED
% ptCloud2 = (ptCloud2 - relpos) * ((rotationM)^-1);

% figure(1)
% hold on 
% plot3(ptCloud10(:,1),ptCloud10(:,2),ptCloud10(:,3),'.')

% Plot ptCloud1 and translated ptCloud10 using plot3 function
grayColor = [.7 .7 .7];
figure(1)
plot3(ptCloud1(:,1),ptCloud1(:,2),ptCloud1(:,3),'.')%, 'Color', grayColor)
hold on
plot3(ptCloud10(:,1),ptCloud10(:,2),ptCloud10(:,3),'.')%, 'Color', grayColor)
% legend("Original", "Second Scan","Matched Scene")

% ******************GRAPHING VOXEL BOUNDARIES************************
v_az = 18;
v_elev = 3;
for i = 1:v_az
    for j = 1:v_elev
        azinum = i*(360/v_az);
        % If only plotting one voxel boundary 
        % azinum = 150;
        elevnum = -22-(40/v_elev) + (40/v_elev)*j;
        % If only plotting on voxel boundary 
        % elevnum = 12;
        rho = [1; 100];
        Az = [azinum; azinum]*pi/180;
        El = [elevnum; elevnum]*pi/180;
        X = rho.*cos(Az).*cos(El);
        Y = rho.*sin(Az).*cos(El);
        Z = rho.*sin(El);
        figure(1)
        hold on 
        plot3(X,Y,Z, 'black', 'LineStyle','--')
    end
end

% Define edge endpoints in polar coordinates
rho = [1; 100];
Az = [150; 150]*pi/180;
El = [-4; -4]*pi/180;

% Convert endpoints to Cartesian coordinates & plot
X = rho.*cos(Az).*cos(El);
Y = rho.*sin(Az).*cos(El);
Z = rho.*sin(El);

% figure(3)
% hold on 
% plot3(X,Y,Z, 'r')

% Define edge endpoints in polar coordinates
rho = [1; 100];
Az = [150; 150]*pi/180;
El = [4; 4]*pi/180;

% Convert endpoints to Cartesian coordinates & plot
X = rho.*cos(Az).*cos(El);
Y = rho.*sin(Az).*cos(El);
Z = rho.*sin(El);

% figure(3)
% hold on 
% plot3(X,Y,Z, 'r')

% Define edge endpoints in polar coordinates
rho = [1; 100];
Az = [160; 160]*pi/180;
El = [4; 4]*pi/180;

% Convert endpoints to Cartesian coordinates & plot
X = rho.*cos(Az).*cos(El);
Y = rho.*sin(Az).*cos(El);
Z = rho.*sin(El);

% figure(3)
% hold on 
% plot3(X,Y,Z, 'r')

% Define edge endpoints in polar coordinates
rho = [1; 100];
Az = [160; 160]*pi/180;
El = [-4; -4]*pi/180;

% Convert endpoints to Cartesian coordinates & plot
X = rho.*cos(Az).*cos(El);
Y = rho.*sin(Az).*cos(El);
Z = rho.*sin(El);

% figure(3)
% hold on 
% plot3(X,Y,Z, 'r')


% ******************** POLAR COORDS ******************************
% Convert cartesian coordiates to polar coordinates
% After translation, before NDT scan matching algorithm
initial_o = [0 0 0];
x_change = tform_ndt_tran(1,1);
y_change = tform_ndt_tran(1,2);
final_o = [x_change y_change 0];
% Define an origin in the middle of initial and final 0
mid_o = [x_change/2 y_change/2 0];

ptCloud1_pol = conv_to_polar(ptCloud1, initial_o);
ptCloud10_pol = conv_to_polar(ptCloud10, initial_o);

ptCloud1_pol_B = conv_to_polar(ptCloud1, final_o);
ptCloud10_pol_B = conv_to_polar(ptCloud10, final_o);

%**************** VOXEL DISTRIBUTION ***********************

% CARTESIAN ********
% Dividing voxels in a cartesian grid
% Using the minimum and maximum limits for each cloud 

% Static cloud
x_min_s = staticCloud.XLimits(1,1);
y_min_s = staticCloud.YLimits(1,1);
z_min_s = staticCloud.ZLimits(1,1);
x_max_s = staticCloud.XLimits(1,2);
y_max_s = staticCloud.YLimits(1,2);
z_max_s = staticCloud.ZLimits(1,2);

% Moving Cloud
x_min_m = movingCloud.XLimits(1,1);
y_min_m = movingCloud.YLimits(1,1);
z_min_m = movingCloud.ZLimits(1,1);
x_max_m = movingCloud.XLimits(1,2);
y_max_m = movingCloud.YLimits(1,2);
z_max_m = movingCloud.ZLimits(1,2);

x_min_total = abs(min(x_min_s, x_min_m));
y_min_total = abs(min(y_min_s, y_min_m));
z_min_total = abs(min(z_min_s, z_min_m));
x_max_total = abs(max(x_max_s, x_max_m));
y_max_total = abs(max(y_max_s, y_max_m));
z_max_total = abs(max(z_max_s, z_max_m));


L = x_max_total + x_min_total;
W = y_max_total + y_min_total;
H = z_max_total + z_min_total;


% Determine side length of the voxel
v_side = 1; %0.5
volume = L*W*H;
v_number = ceil(volume/v_side);

% Determine amount of voxels in length, width, and height
v_L = ceil(L/v_side);
v_W = ceil(W/v_side);
v_H = ceil(H/v_side);

% Add 2 additional voxels to support error of NDT (1 should suffice)
% Potentially just add 1 meter?

Voxels1_cart = cart_voxel_dist(ptCloud1, v_side, L, W, H, x_min_total, y_min_total, z_min_total);
Voxels10_cart = cart_voxel_dist(ptCloud10, v_side, L, W, H, x_min_total, y_min_total, z_min_total);

% SPHERICAL ********
% Dividing voxels in a spherical grid as opposed to cartesian 
% Set parameters of the voxel distribution
top_elev = 20;
bot_elev = -20;

elev_dif = top_elev - bot_elev;

% Degrees of arclength of voxel  
arc_d = 10; %3.6 %36 %10 
% Amount of voxels divided around circumference
v_az = ceil(360/arc_d);
% Degrees per band elevation
dpb = 2;
% Determine amount of bands per azimuth voxel
band_des = 4;
band_amt = band_des*dpb;
% Determine amount of voxels through elevation
% NOTE: for VLP-16, 1 band every 2 degrees 
v_elev = ceil(elev_dif/band_amt);

% Define amount below horizontal which elevation reaches
zero_el = 0 - bot_elev;

Voxels1 = sph_vox_dist(ptCloud1_pol, arc_d, band_amt, elev_dif, zero_el);
Voxels10 = sph_vox_dist(ptCloud10_pol, arc_d, band_amt, elev_dif, zero_el);

% REPEAT VOXEL DISTRIBUTION for polar clouds with center at position B
Voxels1_B = sph_vox_dist(ptCloud1_pol_B, arc_d, band_amt, elev_dif, zero_el);
Voxels10_B = sph_vox_dist(ptCloud10_pol_B, arc_d, band_amt, elev_dif, zero_el);


% ************************ VEHICLE REMOVAL ***************************
% First: seeing impact of removing the vehicle from each pt cloud 
% Determine the range necessary to remove the vehicle from the point cloud
range = 4.1; 
points_rem_1 = rem_vehicle(ptCloud1, ptCloud1_pol, range);
points_rem_10 = rem_vehicle(ptCloud10, ptCloud10_pol, range);

% % Plot points on point cloud to view results
figure(1)
hold on 
plot3(ptCloud1(points_rem_1,1), ptCloud1(points_rem_1,2), ptCloud1(points_rem_1,3), '.', 'Color', '#0072BD')
hold on 
plot3(ptCloud10(points_rem_10,1), ptCloud10(points_rem_10,2), ptCloud10(points_rem_10,3), '.', 'Color', '#D95319')

% Remove designated points from each point cloud

ptCloud1_vrem = ptCloud1;
ptCloud1_vrem(points_rem_1, :) = NaN;
ptCloud10_vrem = ptCloud10;
ptCloud10_vrem(points_rem_10,:) = NaN;

figure(2)
plot3(ptCloud1_vrem(:,1), ptCloud1_vrem(:,2), ptCloud1_vrem(:,3), '.', 'Color', grayColor) %'[0, 0.447, 0.7410]')
hold on 
plot3(ptCloud10_vrem(:,1), ptCloud10_vrem(:,2), ptCloud10_vrem(:,3), '.', 'Color', grayColor) % '[.85, .325, .098]')

% Convert new clouds with vehicle removed to polar clouds
ptCloud1_pol_vrem = conv_to_polar(ptCloud1_vrem, initial_o);
ptCloud10_pol_vrem = conv_to_polar(ptCloud10_vrem, initial_o);

% Convert new clouds with vehicle removed to polar clouds at FINAL POS
ptCloud1_pol_vrem_B = conv_to_polar(ptCloud1_vrem, final_o);
ptCloud10_pol_vrem_B = conv_to_polar(ptCloud10_vrem, final_o);

% Repeat voxel distribution for point clouds with vehicle removed
Voxels1_vrem = sph_vox_dist(ptCloud1_pol_vrem, arc_d, band_amt, elev_dif, zero_el);
Voxels10_vrem = sph_vox_dist(ptCloud10_pol_vrem, arc_d, band_amt, elev_dif, zero_el);



% % *************** DOMAIN BOUNDARY ERROR REMOVAL ************************
% % Remove the points which contribute to domain boundary errors
% % Includes ground plane points inside the blind spot of lidar from each
% % view, as well as vertical limits at tops of scene 
% NOTE: Using clouds with vehicle already removed

% Completing only for spherical voxels

height = 0;
min_angle = -15;
max_angle = 15;

% Analysis 1: remove points from ptCloud 1
remove_pts_lower_1 = domain_error_below(ptCloud1_pol_vrem_B, height, min_angle);
% Plot points to be removed from Cloud 1
figure(2)
hold on 
plot3(ptCloud1(remove_pts_lower_1,1),ptCloud1(remove_pts_lower_1,2),ptCloud1(remove_pts_lower_1,3),'.', 'Color', '#0072BD')

remove_pts_upper_1 = domain_error_above(ptCloud1_pol_vrem_B, height, max_angle);
% Plot points to be removed from Cloud 1
figure(2)
hold on 
plot3(ptCloud1(remove_pts_upper_1,1),ptCloud1(remove_pts_upper_1,2),ptCloud1(remove_pts_upper_1,3),'.', 'Color', '#0072BD')

% Analysis 10: remove points from ptCloud 10
remove_pts_lower_10 = domain_error_below(ptCloud10_pol_vrem, height, min_angle);
% Plot points to be removed from Cloud 10
figure(2)
hold on 
plot3(ptCloud10(remove_pts_lower_10,1),ptCloud10(remove_pts_lower_10,2),ptCloud10(remove_pts_lower_10,3),'.', 'Color', '#D95319')

remove_pts_upper_10 = domain_error_above(ptCloud10_pol_vrem, height, max_angle);
% Plot points to be removed from Cloud 10
figure(2) 
hold on 
plot3(ptCloud10(remove_pts_upper_10,1),ptCloud10(remove_pts_upper_10,2),ptCloud10(remove_pts_upper_10,3),'.', 'Color', '#D95319')

% Remove points from clouds 
ptCloud1_domrem = ptCloud1_vrem;
ptCloud1_domrem(remove_pts_lower_1,:) = NaN;
ptCloud1_domrem(remove_pts_upper_1,:) = NaN;

ptCloud10_domrem = ptCloud10_vrem;
ptCloud10_domrem(remove_pts_lower_10,:) = NaN;
ptCloud10_domrem(remove_pts_upper_10,:) = NaN;

% PLOT NEW CLOUDS
figure(3)
plot3(ptCloud1_domrem(:,1), ptCloud1_domrem(:,2), ptCloud1_domrem(:,3), '.')%, 'Color', grayColor)
hold on 
plot3(ptCloud10_domrem(:,1), ptCloud10_domrem(:,2), ptCloud10_domrem(:,3), '.')%, 'Color', grayColor)

total_pts_domrem_1 = cat(2, remove_pts_lower_1, remove_pts_upper_1);
total_pts_domrem_10 = cat(2, remove_pts_lower_10, remove_pts_upper_10);

ptCloud1_pol_domrem = conv_to_polar(ptCloud1_domrem, initial_o);
ptCloud10_pol_domrem = conv_to_polar(ptCloud10_domrem, initial_o);


% *************** SHADOW MITIGATION ******************************

% Implement pre-processing step for shadowing errors
% Initialize a jump paramenter and minimum number of points threshold
% SHADOW MITIGATION CAN BE APPLIED WITH OR WITHOUT DOMAIN BOUNDARY ERROR REMOVAL 
jump = 1;
% jump = 0.75
point_n = 30;
% ANALYSIS 1 - get points to be removed by method of analysis 1
% NOTE: USE THE VEHICLE REMOVED CLOUDS 
points_Cloud1_A1 = shadow_mitig(ptCloud1_pol_domrem, Voxels1_vrem, jump, point_n, arc_d, band_amt, elev_dif, zero_el); % ptCloud1_pol_vrem
points_Cloud10_A1 = shadow_mitig(ptCloud10_pol_domrem, Voxels10_vrem, jump, point_n, arc_d, band_amt, elev_dif, zero_el); % ptCloud2_pol_vrem

points_Cloud1_FIRST = shadow_mitig_first(ptCloud1_pol_vrem, Voxels1_vrem, jump, point_n, arc_d, band_amt, elev_dif, zero_el);
points_Cloud10_FIRST = shadow_mitig_first(ptCloud10_pol_vrem, Voxels10_vrem, jump, point_n, arc_d, band_amt, elev_dif, zero_el);


% % COMMENTING OUT ANALYSIS 2 UNTIL ANALYSIS ONE SOLVED AND COMPLETED
% % ANALYSIS 2 - get points to be removed by method of analysis 2
% % First, need new polar coordinates with center at ptcloud before alignment
% % Also, need new voxel distribution 
% 
% points_Cloud1_A2 = shadow_mitig(ptCloud1_pol_B, Voxels1_B, jump, point_n, arc_d, band_amt, elev_dif, zero_el);
% points_Cloud10_A2 = shadow_mitig(ptCloud10_pol_B, Voxels10_B, jump, point_n, arc_d, band_amt, elev_dif, zero_el);
% 
% % Create an array with the total points removed from A1 and A2
% totalpts_Cloud1 = cat(1, points_Cloud1_A1, points_Cloud1_A2);
% totalpts_Cloud10 = cat(1, points_Cloud10_A1, points_Cloud10_A2);
% 

% Repeat_check_C1 = unique(totalpts_Cloud1);
% Repeat_check_C10 = unique(totalpts_Cloud10);
% L1_C1 = length(totalpts_Cloud1);
% L2_C1 = length(Repeat_check_C1);
% L1_C10 = length(totalpts_Cloud10);
% L2_C10= length(Repeat_check_C10);
% pts_repeated_C1 = L1_C1-L2_C1;
% pts_repeated_C10 = L1_C10-L2_C10;

% SHOW POINTS REMOVED BY SHADOW MITIGATION IN REAL SCENE
figure(3)
hold on 
plot3(ptCloud1(points_Cloud1_A1,1), ptCloud1(points_Cloud1_A1,2), ptCloud1(points_Cloud1_A1,3), '.', 'Color', '#0072BD')
hold on 
plot3(ptCloud10(points_Cloud10_A1,1), ptCloud10(points_Cloud10_A1,2), ptCloud10(points_Cloud10_A1,3), '.', 'Color', '#D95319')

% % SHOW POINT REMOVED BY NEW SHADOW MITIGATION IN REAL SCENE
% figure(3)
% plot3(ptCloud1_vrem(:,1), ptCloud1_vrem(:,2), ptCloud1_vrem(:,3), '.')
% hold on 
% plot3(ptCloud10_vrem(:,1), ptCloud10_vrem(:,2), ptCloud10_vrem(:,3), '.')
% hold on 
% plot3(ptCloud1(points_Cloud1_FIRST,1), ptCloud1(points_Cloud1_FIRST,2), ptCloud1(points_Cloud1_FIRST,3), '.')
% hold on 
% plot3(ptCloud10(points_Cloud10_FIRST,1), ptCloud10(points_Cloud10_FIRST,2), ptCloud10(points_Cloud10_FIRST,3), '.')




% LOOKING AT SPECIFIC VOXELS 


% figure(3)
% plot3(ptCloud1_vrem(:,1), ptCloud1_vrem(:,2), ptCloud1_vrem(:,3), '.', 'Color', [.7 .7 .7])
% hold on 
% plot3(ptCloud10_vrem(:,1), ptCloud10_vrem(:,2), ptCloud10_vrem(:,3), '.', 'Color', [128 128 128]/255)
% hold on 
% plot3(ptCloud1_vrem(Voxels1{34,3},1),ptCloud1_vrem(Voxels1{34,3},2), ptCloud1_vrem(Voxels1{34,3},3), '.', 'Color', '#0072BD')
% hold on 
% plot3(ptCloud10_vrem(Voxels10{34,3},1),ptCloud10_vrem(Voxels10{34,3},2), ptCloud10_vrem(Voxels10{34,3},3), '.', 'Color', '#D95319')

% figure(3)
% hold on 
% plot3(ptCloud1_vrem(points_Cloud1_A1(Voxels1{18,3},1)),ptCloud1_vrem(points_Cloud1_A1(Voxels1{18,3},2)), ptCloud1_vrem(points_Cloud1_A1(Voxels1{18,3},3), '.')
% hold on 
% plot3(ptCloud10_vrem(points_Cloud10_A1(Voxels10{18,3},1)),ptCloud10_vrem(points_Cloud10_A1(Voxels10{18,3},2)), ptCloud10_vrem(points_Cloud10_A1(Voxels10{18,3},3), '.')

%******************************************************

% Remove points from each point cloud for both analysis 1 and analysis 2
ptCloud1_A1 = ptCloud1_vrem;
ptCloud1_A1(points_Cloud1_A1,:) = NaN;
ptCloud10_A1 = ptCloud10_vrem;
ptCloud10_A1(points_Cloud10_A1,:) = NaN;


% % COMMENTING OUT ANALYSIS 2 UNTIL ANALYSIS ONE SOLVED AND COMPLETED
% ptCloud1_A2 = ptCloud1_vrem;
% ptCloud1_A2(points_Cloud1_A2,:) = NaN;
% ptCloud10_A2 = ptCloud10_vrem;
% ptCloud10_A2(points_Cloud10_A2,:) = NaN;

ptCloud1_A1NEW = ptCloud1_vrem;
ptCloud1_A1NEW(points_Cloud1_FIRST,:) = NaN;
ptCloud10_A1NEW = ptCloud10_vrem;
ptCloud10_A1NEW(points_Cloud10_FIRST,:) = NaN;


% LOOKING AT SPECIFIC VOXELS 

% figure(2)
% hold on 
% plot3(ptCloud1_vrem(Voxels1{18,3},1),ptCloud1_vrem(Voxels1{18,3},2), ptCloud1_vrem(Voxels1{18,3},3), '.', 'color', 'm')
% hold on 
% plot3(ptCloud10_vrem(Voxels10{18,3},1),ptCloud10_vrem(Voxels10{18,3},2), ptCloud10_vrem(Voxels10{18,3},3), '.', 'color', 'g')

% figure(3)
% hold on 
% plot3(ptCloud1_A1(Voxels1{34,3},1),ptCloud1_A1(Voxels1{34,3},2), ptCloud1_A1(Voxels1{34,3},3), '.')
% hold on 
% plot3(ptCloud10_A1(Voxels10{34,3},1),ptCloud10_A1(Voxels10{34,3},2), ptCloud10_A1(Voxels10{34,3},3), '.')

% figure(4)
% plot3(ptCloud1_A1(Voxels1{18,2},1),ptCloud1_A1(Voxels1{18,2},2), ptCloud1_A1(Voxels1{18,2},3), '.', 'Color', '#0072BD')
% hold on 
% plot3(ptCloud10_A1(Voxels10{18,2},1),ptCloud10_A1(Voxels10{18,2},2), ptCloud10_A1(Voxels10{18,2},3), '.', 'Color', '#D95319')

% % Create clouds with both analyses removed
% % COMMENTING OUT ANALYSIS 2 UNTIL ANALYSIS ONE SOLVED AND COMPLETED
% ptCloud1_A1_A2 = ptCloud1;
% ptCloud10_A1_A2 = ptCloud10;
% ptCloud1_A1_A2(totalpts_Cloud1,:) = NaN;
% ptCloud10_A1_A2(totalpts_Cloud10,:) = NaN;

% figure(1)
% hold on 
% plot3(ptCloud1(totalpts_Cloud1,1), ptCloud1(totalpts_Cloud1,2), ptCloud1(totalpts_Cloud1,3), '.')
% hold on 
% plot3(ptCloud10(totalpts_Cloud10,1), ptCloud10(totalpts_Cloud10,2), ptCloud10(totalpts_Cloud10,3), '.')

% ********************************************************

% Plot new point clouds after shadow mitigation analysis 1
% CHECK FIGURE NUMBERS
% ORIGINAL SHADOW MITIGATION 
figure(4)
plot3(ptCloud1_A1(:,1),ptCloud1_A1(:,2),ptCloud1_A1(:,3),'.')
hold on
plot3(ptCloud10_A1(:,1),ptCloud10_A1(:,2),ptCloud10_A1(:,3),'.')
title('Shadow Mitig. Analysis 1 removed')

% NEW SHADOW MITIGATION 
figure(5)
plot3(ptCloud1_A1NEW(:,1),ptCloud1_A1NEW(:,2),ptCloud1_A1NEW(:,3),'.')
hold on
plot3(ptCloud10_A1NEW(:,1),ptCloud10_A1NEW(:,2),ptCloud10_A1NEW(:,3),'.')
title('Shadow Mitig. NEW METHOD')



% 
% 
% % PLOT NEW POINT CLOUDS AFTER SHADOW MITIGATION analysis 2
% figure(6)
% plot3(ptCloud1_A2(:,1),ptCloud1_A2(:,2),ptCloud1_A2(:,3),'.')
% hold on
% plot3(ptCloud2_A2(:,1),ptCloud2_A2(:,2),ptCloud2_A2(:,3),'.')
% title('Shadow Mitig. Analysis 2 removed')

% % PLOT NEW POINT CLOUDS COMBINING BOTH SHADOW MITIGATION ANALYSES
% figure(3)
% plot3(ptCloud1_A1_A2(:,1),ptCloud1_A1_A2(:,2),ptCloud1_A1_A2(:,3),'.')
% hold on 
% plot3(ptCloud10_A1_A2(:,1),ptCloud10_A1_A2(:,2),ptCloud10_A1_A2(:,3),'.')
% title('Shadow Mitig. Analysis 1 AND 2 removed')



% ************* COMBINING DOMAIN BOUNDARY ERROR AND SHADOW MITIG.*********
% Create a new point cloud 1 and point cloud two for the combined shadow
% mitigation and domain boundary error removal
% ***NOTE: ONLY USING ANALYSIS 1 HERE
% *** IMPLEMENT ANALYSIS 2 LATER USING "totalpts_CloudXX"
% Pt Cloud 1
ptCloud1_DR_shad = ptCloud1_domrem;
ptCloud1_DR_shad(points_Cloud1_A1,:) = NaN;
% Pt Cloud 10
ptCloud10_DR_shad = ptCloud10_domrem;
ptCloud10_DR_shad(points_Cloud10_A1,:) = NaN;

% Combining DOMAIN BOUNDARY, VR, AND SHADOW MITIG
% Using new shadow mitigation algorithm (FIRST SET OF POINTS)
% Pt Cloud 1
ptCloud1_DR_shad1 = ptCloud1_domrem;
ptCloud1_DR_shad1(points_Cloud1_A1,:) = NaN;
% Pt Cloud 10
ptCloud10_DR_shad1 = ptCloud10_domrem;
ptCloud10_DR_shad1(points_Cloud10_FIRST,:) = NaN;



% Voxel distribution for polar clouds with shadow mitigation 
% FIRST: create new polar point cloud after shadow mitigation 
% ***NOTE: ONLY USING ANALYSIS 1 HERE
% *** IMPLEMENT ANALYSIS 2 LATER USING "ptCloudXX_A1_A2"
ptCloud1_pol_shad = conv_to_polar(ptCloud1_A1, initial_o);
ptCloud10_pol_shad = conv_to_polar(ptCloud10_A1, initial_o);
ptCloud1_pol_shad_F = conv_to_polar(ptCloud1_A1NEW, initial_o);
ptCloud10_pol_shad_F = conv_to_polar(ptCloud10_A1NEW, initial_o);
Voxels1_shad = sph_vox_dist(ptCloud1_pol_shad, arc_d, band_amt, elev_dif, zero_el);
Voxels10_shad = sph_vox_dist(ptCloud10_pol_shad, arc_d, band_amt, elev_dif, zero_el);
Voxels1_shad_FIRST = sph_vox_dist(ptCloud1_pol_shad_F, arc_d, band_amt, elev_dif, zero_el);
Voxels10_shad_FIRST = sph_vox_dist(ptCloud10_pol_shad_F, arc_d, band_amt, elev_dif, zero_el);
Voxels1_domrem = sph_vox_dist(ptCloud1_pol_domrem, arc_d, band_amt, elev_dif, zero_el);
Voxels10_domrem = sph_vox_dist(ptCloud10_pol_domrem, arc_d, band_amt, elev_dif, zero_el);

% Voxel distribution for polar clouds with shadow mitigation AND domain boundary removal 
% ********* $$$$$ POTENTIALLY CHANGE INITIAL COORDS $$$$$ **********
% FIRST: create new polar point cloud after combination of domrem and shad.
ptCloud1_pol_DR_shad = conv_to_polar(ptCloud1_DR_shad, initial_o); %mid_o
ptCloud10_pol_DR_shad = conv_to_polar(ptCloud10_DR_shad, initial_o); %mid_o
ptCloud1_pol_domrem_shad = conv_to_polar(ptCloud1_DR_shad, initial_o);
ptCloud10_pol_domrem_shad = conv_to_polar(ptCloud10_DR_shad, initial_o);

Voxels1_VR_shad = sph_vox_dist(ptCloud1_pol_DR_shad, arc_d, band_amt, elev_dif, zero_el);
Voxels10_VR_shad = sph_vox_dist(ptCloud10_pol_DR_shad, arc_d, band_amt, elev_dif, zero_el);
Voxels1_DR_shad = sph_vox_dist(ptCloud1_pol_domrem_shad, arc_d, band_amt, elev_dif, zero_el);
Voxels10_DR_shad = sph_vox_dist(ptCloud10_pol_domrem_shad, arc_d, band_amt, elev_dif, zero_el);


% % Make a plot to visualize the points removed from domain boundary and ...
% % ... shadow mitigation 
% figure(4)
% plot3(ptCloud1(:,1),ptCloud1(:,2),ptCloud1(:,3),'.')
% hold on 
% plot3(ptCloud2(:,1),ptCloud2(:,2),ptCloud2(:,3),'.')
% hold on 
% % Domain Boundary 
% plot3(ptCloud1(total_pts_domrem_1,1),ptCloud1(total_pts_domrem_1,2),ptCloud1(total_pts_domrem_1,3),'.')
% hold on
% plot3(ptCloud2(total_pts_domrem_2,1),ptCloud2(total_pts_domrem_2,2),ptCloud2(total_pts_domrem_2,3),'.')
% hold on 
% % Shadow Mitigation 
% plot3(ptCloud1(totalpts_Cloud1,1),ptCloud1(totalpts_Cloud1,2),ptCloud1(totalpts_Cloud1,3),'.')
% hold on 
% plot3(ptCloud2(totalpts_Cloud2,1),ptCloud2(totalpts_Cloud2,2),ptCloud2(totalpts_Cloud2,3),'.')
% 
% Make a plot for points REMOVED (shadow mitig. and domain boundary)
figure(6)
plot3(ptCloud1_DR_shad(:,1), ptCloud1_DR_shad(:,2), ptCloud1_DR_shad(:,3), '.')
hold on 
plot3(ptCloud10_DR_shad(:,1), ptCloud10_DR_shad(:,2), ptCloud10_DR_shad(:,3), '.')
%*******************************************************************************







% Plotting voxels that show the difference between the original largest errors 
% % Before shaodw mitigation 
% figure(4)
% plot3(ptCloud1(Voxels1{23,1},1),ptCloud1(Voxels1{23,1},2), ptCloud1(Voxels1{23,1},3), '.')
% hold on 
% plot3(ptCloud10(Voxels10{23,1},1),ptCloud10(Voxels10{23,1},2), ptCloud10(Voxels10{23,1},3), '.')
% % AFTER shadow mitigation
% figure(5)
% plot3(ptCloud1_A1_A2(Voxels1{23,1},1),ptCloud1_A1_A2(Voxels1{23,1},2), ptCloud1_A1_A2(Voxels1{23,1},3), '.')
% hold on 
% plot3(ptCloud10_A1_A2(Voxels10{23,1},1),ptCloud10_A1_A2(Voxels10{23,1},2), ptCloud10_A1_A2(Voxels10{23,1},3), '.')
% 
% figure(2)
% hold on 
% plot3(ptCloud1_vrem(Voxels1_vrem{19,3},1),ptCloud1_vrem(Voxels1_vrem{19,3},2), ptCloud1_vrem(Voxels1_vrem{19,3},3), '.', 'Color', '#EDB120')
% hold on 
% plot3(ptCloud10_vrem(Voxels10_vrem{19,3},1),ptCloud10_vrem(Voxels10_vrem{19,3},2), ptCloud10_vrem(Voxels10_vrem{19,3},3), '.', 'Color', '#7E2F8E')
% 
% % 
% figure(3)
% plot3(ptCloud1_vrem(Voxels1_vrem{7,1},1),ptCloud1_vrem(Voxels1_vrem{7,1},2), ptCloud1_vrem(Voxels1_vrem{7,1},3), '.')
% hold on 
% plot3(ptCloud10_vrem(Voxels10_vrem{7,1},1),ptCloud10_vrem(Voxels10_vrem{7,1},2), ptCloud10_vrem(Voxels10_vrem{7,1},3), '.')

% figure(1)
% plot3(ptCloud1(Voxels1{5,2},1),ptCloud1(Voxels1{5,2},2), ptCloud1(Voxels1{5,2},3), '.')
% hold on 
% plot3(ptCloud10(Voxels10{5,2},1),ptCloud10(Voxels10{5,2},2), ptCloud10(Voxels10{5,2},3), '.')

% *************** Mean calculations ************************
% % *** CARTESIAN 
means1_cart = calc_means_cart(Voxels1_cart, ptCloud1, v_L, v_W, v_H);
means10_cart = calc_means_cart(Voxels10_cart, ptCloud10, v_L, v_W, v_H);

% *** SPHERICAL
% Use either the ORIGINAL OR NEW POINT CLOUDS
means1 = calc_means_sph(Voxels1, ptCloud1, v_az, v_elev);
means10 = calc_means_sph(Voxels10, ptCloud10, v_az, v_elev);

% *** Vehicle points removed 
means1_vrem = calc_means_sph(Voxels1_vrem, ptCloud1_vrem, v_az, v_elev);
means10_vrem = calc_means_sph(Voxels10_vrem, ptCloud10_vrem, v_az, v_elev);
% 
% *** Shadow Mitigation 
% NOTE USING ONLY ANALYSIS 1
% NOTE VEHICLE IS REMOVED AS WELL (makes next quiver unnecessary)
means1_shad = calc_means_sph(Voxels1_shad, ptCloud1_A1, v_az, v_elev);
means10_shad = calc_means_sph(Voxels10_shad, ptCloud10_A1, v_az, v_elev);

% % *** Shadow Mitigation and Domain Boundary error pts removed
% means1_VR_shad = calc_means_sph(Voxels1_VR_shad, ptCloud1_VR_shad, v_az, v_elev);
% means10_VR_shad = calc_means_sph(Voxels10_VR_shad, ptCloud10_VR_shad, v_az, v_elev);

% *** NEW Shadow Mitigation - first group of points 
means1_shad_F = calc_means_sph(Voxels1_shad_FIRST, ptCloud1_A1NEW, v_az, v_elev);
means10_shad_F = calc_means_sph(Voxels10_shad_FIRST, ptCloud10_A1NEW, v_az, v_elev);

% *** DOMAIN BOUNDARY removal with vehicle removal
means1_domrem = calc_means_sph(Voxels1_domrem, ptCloud1_domrem, v_az, v_elev);
means10_domrem = calc_means_sph(Voxels10_domrem, ptCloud10_domrem, v_az, v_elev);

% *** DOMAIN BOUNDARY and SHADOW MITIG
means1_domrem_shad = calc_means_sph(Voxels1_DR_shad, ptCloud1_DR_shad, v_az, v_elev);
means10_domrem_shad = calc_means_sph(Voxels10_DR_shad, ptCloud10_DR_shad, v_az, v_elev);


means_dif_cart = calc_mean_dif_cart(means1_cart, means10_cart, Voxels1_cart, Voxels10_cart, v_L, v_W, v_H);
means_dif = calc_mean_dif_sph(means1, means10, Voxels1, Voxels10, v_az, v_elev, point_n);
means_dif_vrem = calc_mean_dif_sph(means1_vrem, means10_vrem, Voxels1_vrem, Voxels10_vrem, v_az, v_elev, point_n);
means_dif_shad = calc_mean_dif_sph(means1_shad, means10_shad, Voxels1_shad, Voxels10_shad, v_az, v_elev, point_n);
means_dif_shad_F = calc_mean_dif_sph(means1_shad_F, means10_shad_F, Voxels1_shad_FIRST, Voxels10_shad_FIRST, v_az, v_elev, point_n);
% means_dif_VR_shad = calc_mean_dif_sph(means1_VR_shad, means10_VR_shad, Voxels1_VR_shad, Voxels10_VR_shad, v_az, v_elev, point_n);
means_dif_domrem = calc_mean_dif_sph(means1_domrem, means10_domrem, Voxels1_domrem, Voxels10_domrem, v_az, v_elev, point_n);
means_dif_DR_shad = calc_mean_dif_sph(means1_domrem_shad, means10_domrem_shad, Voxels1_DR_shad, Voxels10_DR_shad, v_az, v_elev, point_n);

% *************** Quiver Plot ******************************
% Define total number of voxels 
% Change which means array based on desired starting location 
% Define positions for desired location 
pos_x = 1; 
pos_y = 2;
pos_z = 3;

% CARTESIAN ****
vox_number_cart = v_L*v_W*v_H;
X_q_cart = quiver_setup_cart(vox_number_cart, means1_cart, v_L, v_W, v_H, pos_x);
Y_q_cart = quiver_setup_cart(vox_number_cart, means1_cart, v_L, v_W, v_H, pos_y);
Z_q_cart = quiver_setup_cart(vox_number_cart, means1_cart, v_L, v_W, v_H, pos_z);
U_q_cart = quiver_setup_cart(vox_number_cart, means_dif_cart, v_L, v_W, v_H, pos_x);
V_q_cart = quiver_setup_cart(vox_number_cart, means_dif_cart, v_L, v_W, v_H, pos_y);
W_q_cart = quiver_setup_cart(vox_number_cart, means_dif_cart, v_L, v_W, v_H, pos_z);

% SPHERICAL ****
% Set total number of voxels
vox_number = v_az*v_elev;
X_q = quiver_setup_sph(vox_number, means1, v_az, v_elev, pos_x);
Y_q = quiver_setup_sph(vox_number, means1, v_az, v_elev, pos_y);
Z_q = quiver_setup_sph(vox_number, means1, v_az, v_elev, pos_z);
U_q = quiver_setup_sph(vox_number, means_dif, v_az, v_elev, pos_x);
V_q = quiver_setup_sph(vox_number, means_dif, v_az, v_elev, pos_y);
W_q = quiver_setup_sph(vox_number, means_dif, v_az, v_elev, pos_z);

% VEHICLE REMOVED FROM SCENE ***
X_q_VR = quiver_setup_sph(vox_number, means1_vrem, v_az, v_elev, pos_x);
Y_q_VR = quiver_setup_sph(vox_number, means1_vrem, v_az, v_elev, pos_y);
Z_q_VR = quiver_setup_sph(vox_number, means1_vrem, v_az, v_elev, pos_z);
U_q_VR = quiver_setup_sph(vox_number, means_dif_vrem, v_az, v_elev, pos_x);
V_q_VR = quiver_setup_sph(vox_number, means_dif_vrem, v_az, v_elev, pos_y);
W_q_VR = quiver_setup_sph(vox_number, means_dif_vrem, v_az, v_elev, pos_z);
% 
% SHADOW MITIGATION ERRORS REMOVED **
X_q_shad = quiver_setup_sph(vox_number, means1_shad, v_az, v_elev, pos_x);
Y_q_shad = quiver_setup_sph(vox_number, means1_shad, v_az, v_elev, pos_y);
Z_q_shad = quiver_setup_sph(vox_number, means1_shad, v_az, v_elev, pos_z);
U_q_shad = quiver_setup_sph(vox_number, means_dif_shad, v_az, v_elev, pos_x);
V_q_shad = quiver_setup_sph(vox_number, means_dif_shad, v_az, v_elev, pos_y);
W_q_shad = quiver_setup_sph(vox_number, means_dif_shad, v_az, v_elev, pos_z);
% 
% % VEHICLE REMOVED AND SHADOW MITIGATION ERRORS REMOVED
% X_q_VR_shad = quiver_setup_sph(vox_number, means1_VR_shad, v_az, v_elev, pos_x);
% Y_q_VR_shad = quiver_setup_sph(vox_number, means1_VR_shad, v_az, v_elev, pos_y);
% Z_q_VR_shad = quiver_setup_sph(vox_number, means1_VR_shad, v_az, v_elev, pos_z);
% U_q_VR_shad = quiver_setup_sph(vox_number, means_dif_VR_shad, v_az, v_elev, pos_x);
% V_q_VR_shad = quiver_setup_sph(vox_number, means_dif_VR_shad, v_az, v_elev, pos_y);
% W_q_VR_shad = quiver_setup_sph(vox_number, means_dif_VR_shad, v_az, v_elev, pos_z);

% SHADOW MITIGATION NEW VERSION (FOCUS ON FIRST POINTS)
X_q_shad_F = quiver_setup_sph(vox_number, means1_shad_F, v_az, v_elev, pos_x);
Y_q_shad_F = quiver_setup_sph(vox_number, means1_shad_F, v_az, v_elev, pos_y);
Z_q_shad_F = quiver_setup_sph(vox_number, means1_shad_F, v_az, v_elev, pos_z);
U_q_shad_F = quiver_setup_sph(vox_number, means_dif_shad_F, v_az, v_elev, pos_x);
V_q_shad_F = quiver_setup_sph(vox_number, means_dif_shad_F, v_az, v_elev, pos_y);
W_q_shad_F = quiver_setup_sph(vox_number, means_dif_shad_F, v_az, v_elev, pos_z);

% DOMAIN BOUNDARY REMOBVAL (with vehicle removed)
X_q_DR = quiver_setup_sph(vox_number, means1_domrem, v_az, v_elev, pos_x);
Y_q_DR = quiver_setup_sph(vox_number, means1_domrem, v_az, v_elev, pos_y);
Z_q_DR = quiver_setup_sph(vox_number, means1_domrem, v_az, v_elev, pos_z);
U_q_DR = quiver_setup_sph(vox_number, means_dif_domrem, v_az, v_elev, pos_x);
V_q_DR = quiver_setup_sph(vox_number, means_dif_domrem, v_az, v_elev, pos_y);
W_q_DR = quiver_setup_sph(vox_number, means_dif_domrem, v_az, v_elev, pos_z);

% DOMAIN BOUNDARY and SHADOW MITIGATION (with vehicle removed)
X_q_DR_shad = quiver_setup_sph(vox_number, means1_domrem_shad, v_az, v_elev, pos_x);
Y_q_DR_shad = quiver_setup_sph(vox_number, means1_domrem_shad, v_az, v_elev, pos_y);
Z_q_DR_shad = quiver_setup_sph(vox_number, means1_domrem_shad, v_az, v_elev, pos_z);
U_q_DR_shad = quiver_setup_sph(vox_number, means_dif_DR_shad, v_az, v_elev, pos_x);
V_q_DR_shad = quiver_setup_sph(vox_number, means_dif_DR_shad, v_az, v_elev, pos_y);
W_q_DR_shad = quiver_setup_sph(vox_number, means_dif_DR_shad, v_az, v_elev, pos_z);
% 

% % Form quiver plot 
% figure(4)
% quiver3(X_q_cart, Y_q_cart, Z_q_cart, U_q_cart*10, V_q_cart*10, W_q_cart*10, 'AutoScale', 'off')

% CHANGE FIGURE NUMBERS TO MATCH SEQUENTIAL ORDER
figure(6)
quiver3(X_q, Y_q, Z_q, U_q, V_q, W_q, 'AutoScale', 'off')
% axis equal

figure(7)
quiver3(X_q_VR, Y_q_VR, Z_q_VR, U_q_VR, V_q_VR, W_q_VR, 'AutoScale', 'off')
title("Vector Plot with Vehicle removed")

figure(8)
quiver3(X_q_shad, Y_q_shad, Z_q_shad, U_q_shad, V_q_shad, W_q_shad, 'AutoScale', 'off')
title("Vector Plot with shadow mitigation algorithm applied")

% figure(8)
% quiver3(X_q_shad_F, Y_q_shad_F, Z_q_shad_F, U_q_shad_F, V_q_shad_F, W_q_shad_F, 'AutoScale', 'off')
% title("Vector Plot with NEW shadow mitigation algorithm applied")

% figure(8)
% quiver3(X_q_VR_shad, Y_q_VR_shad, Z_q_VR_shad, U_q_VR_shad, V_q_VR_shad, W_q_VR_shad, 'AutoScale', 'off')
% title("Vector Plot with domain boundary rem. and shadow mitigation")

figure(9)
quiver3(X_q_DR, Y_q_DR, Z_q_DR, U_q_DR, V_q_DR, W_q_DR, 'AutoScale', 'off')
title("Vector Plot with domain boundary errors removed")

figure(10)
quiver3(X_q_DR_shad, Y_q_DR_shad, Z_q_DR_shad, U_q_DR_shad, V_q_DR_shad, W_q_DR_shad, 'AutoScale', 'off')
title("Domain boundary errors removed and shadow mitigation")

%**********************************************************
% ********************** Functions ************************
% *********************************************************

% Function that takes the PC file and converts it into a point cloud
function ptCloud_i = create_cloud(im_i)
    x_i = im_i(:,:,1);
    y_i = im_i(:,:,2);
    z_i = im_i(:,:,3);
    x_i_s = format_ptCloud(x_i);
    y_i_s = format_ptCloud(y_i);
    z_i_s = format_ptCloud(z_i);
    ptCloud_i = [x_i_s y_i_s z_i_s];
end


% Function that takes values of x (or y or z) double and converts to a
% format usable for ptCloud arithmetic 
% Inputs: 
% 1) Values of point cloud
function ptCloud_num_pos = format_ptCloud(i_1)
    azi_pts = length(i_1);
    elev_pts = height(i_1);
    pt_number = azi_pts*elev_pts;
    ptCloud_num_pos = zeros(pt_number, 1);
    num = 1;
    for i = 1:elev_pts
        for j = 1:azi_pts
            point_i = i_1(i,j);
            ptCloud_num_pos(num) = point_i;
            num = num + 1;
        end
    end
end

% Function that converts the point cloud to polar coordinates 
% Inputs: Point Cloud in cartesian coords, Origin
% Outputs: Polar Point Cloud 
function ptCloud_polar = conv_to_polar(ptCloud, origin)
    ptCloud_polar = zeros(length(ptCloud),3);
    for i = 1:length(ptCloud)
        X = ptCloud(i,1) + origin(1);
        Y = ptCloud(i,2) + origin(2);
        Z = ptCloud(i,3) + origin(3);
        [theta, rho, z] = cart2pol(X,Y,Z);
        ptCloud_polar(i,:) = [theta, rho, z];
        azi_correct = atan2(Y,X);
        ptCloud_polar(i,1) = azi_correct;
        rho_correct = sqrt(X^2 + Y^2 + Z^2);
        ptCloud_polar(i,2) = rho_correct;
        elevation = atan(Z/sqrt(X^2 + Y^2));
        ptCloud_polar(i,3) = elevation;
    end
end

% Function that places cartesian coordinates into cartesian voxels 
% Inputs:
% 1) Cartesian point cloud 
% 2) Voxel side length
% 3) Length of scene 
% 4) Width of scene
% 5) Height of scene
% 6) Minimum x point of the specific scene
% 7) Minimum y point of the specific scene
% 8) Minimum z point of the specific scene
% Outputs: voxel distribution 
function Voxels = cart_voxel_dist(ptCloud, v_side, L, W, H, x_min, y_min, z_min)

    Voxels{ceil(L/v_side),ceil(W/v_side),ceil(H/v_side)} = [];

    for q = 1:length(ptCloud)
        x = ptCloud(q,1);
        y = ptCloud(q,2);
        z = ptCloud(q,3);
        xVoxelInd = ceil((x + x_min)/v_side);
        yVoxelInd = ceil((y + y_min)/v_side);
        zVoxelInd = ceil((z + z_min)/v_side);
        % Added ERROR CHECK for zero value
        if xVoxelInd < 1; xVoxelInd = 1; end
        if yVoxelInd < 1; yVoxelInd = 1; end
        if zVoxelInd < 1; zVoxelInd = 1; end
        % Check if number is NAN
        if isnan(x) | isnan(y) | isnan(z)
        else
            % Add row number from point cloud to cell array for given vox
            Voxels{xVoxelInd, yVoxelInd, zVoxelInd} = [Voxels{xVoxelInd, yVoxelInd, zVoxelInd} q];
        end
    end
end


% Function that places polar coordinates into spherical voxels
% Inputs:
% 1) Polar point cloud
% 2) Arc length per voxel (az)
% 3) Band amount (elev)
% 4) Difference in elevation
% 5) Zero elevation point  
% Outputs: Voxel distribution
function Voxels = sph_vox_dist(ptCloud, arc_d, band_amt, elev_dif, zero_el)
    v_az = ceil(360/arc_d);
    v_elev = ceil(elev_dif/band_amt);
    Voxels{v_az, v_elev} = [];

    for q = 1:length(ptCloud)
        azi = rad2deg(ptCloud(q,1));
        rho = ptCloud(q,2);
        elev = rad2deg(ptCloud(q,3));

        % Create an indice for the azimuth and elevation 
        % NOTE: Theta (azimuth) goes from -pi to pi
        % NOTE: Elevation (elev) goes from specified range
        azVoxelInd = ceil((azi+180)/arc_d);
        elevVoxelInd = ceil((elev + zero_el)/band_amt);
        % if elevVoxelInd>8; keyboard; end
        % Add an error check for zero value 
        if azVoxelInd < 1; azVoxelInd = 1; end
        if elevVoxelInd < 1; elevVoxelInd = 1; end
        % Check if number is a NAN
        if isnan(azi) | isnan(rho) | isnan(elev)
        else
            Voxels{azVoxelInd, elevVoxelInd} = [Voxels{azVoxelInd, elevVoxelInd} q];
        end
    end
end

% Function that implements shadow mitigation algorithm
% Inputs: 
% 1) Polar point cloud
% 2) Array of voxels with distributed pts
% 3) Jump param
% 4) Min pts param
% 5) Arc length per voxel (az)
% 6) Band amount (elev)
% 7) Difference in elevation
% 8) Zero elevation point 
% Outputs: Array of points to be effected by shadow mitigation 
function points_disc = shadow_mitig(ptCloud_pol, Voxels, jump, min_pts, arc_d, band_amt, elev_dif, zero_el)
    points_disc = [];
    V_rad = zeros(length(ptCloud_pol), 1);
    v_az = ceil(360/arc_d);
    v_elev = ceil(elev_dif/band_amt);
    for i = 1:v_az
        for j = 1:v_elev
            az_upper = (i*arc_d)-180;
            az_lower = (i*arc_d)-180 - arc_d;
            elev_upper = (j*band_amt)-zero_el;
            elev_lower = (j*band_amt)-zero_el-band_amt;
            Voxelij = Voxels{i,j};
            radii = ptCloud_pol(Voxelij,2);
            Vij = sort(radii);
            l_min = 1;
            l_max = length(Vij);
            for l = 2:length(Vij)
                diff = Vij(l) - Vij(l-1);
                if diff > jump
                    if l - l_min > min_pts
                        l_max = l-1; 
                        break
                    else
                        l_min = l;

                    end
                end
            end
            % Add a check for empty voxels
            if length(Vij) == 0
            elseif l_max - l_min < min_pts
            else
                r_min = Vij(l_min);
                r_max = Vij(l_max);
                below_rr = find(ptCloud_pol(:,2)<r_min & az_lower<rad2deg(ptCloud_pol(:,1)) & rad2deg(ptCloud_pol(:,1))<az_upper & elev_lower<rad2deg(ptCloud_pol(:,3)) & rad2deg(ptCloud_pol(:,3))<elev_upper);
                % ptCloud_new(below_rr,:) = NaN;
                above_rr = find(ptCloud_pol(:,2)>r_max & az_lower<rad2deg(ptCloud_pol(:,1)) & rad2deg(ptCloud_pol(:,1))<az_upper & elev_lower<rad2deg(ptCloud_pol(:,3)) & rad2deg(ptCloud_pol(:,3))<elev_upper);
                % ptCloud_new(above_rr,:) = NaN;
                points_vox_disc = cat(1, below_rr, above_rr);
                points_disc = cat(1, points_vox_disc, points_disc);
            end
        end
    end
end

% Function that implements a NEW VERSION of the shadow mitigation algorithm
% Inputs: 
% 1) Polar point cloud
% 2) Array of voxels with distributed pts
% 3) Jump param
% 4) Min pts param
% 5) Arc length per voxel (az)
% 6) Band amount (elev)
% 7) Difference in elevation
% 8) Zero elevation point 
% Outputs: Array of points to be effected by shadow mitigation 
function points_disc = shadow_mitig_first(ptCloud_pol, Voxels, jump, min_pts, arc_d, band_amt, elev_dif, zero_el)
    points_disc = [];
    V_rad = zeros(length(ptCloud_pol), 1);
    v_az = ceil(360/arc_d);
    v_elev = ceil(elev_dif/band_amt);
    for i = 1:v_az
        for j = 1:v_elev
            az_upper = (i*arc_d)-180;
            az_lower = (i*arc_d)-180 - arc_d;
            elev_upper = (j*band_amt)-zero_el;
            elev_lower = (j*band_amt)-zero_el-band_amt;
            Voxelij = Voxels{i,j};
            radii = ptCloud_pol(Voxelij,2);
            Vij = sort(radii);
            l_min = 1;
            l_max = length(Vij);
            for l = 2:length(Vij)
                diff = Vij(l) - Vij(l-1);
                if diff > jump
                    if l - l_min > min_pts
                        l_max = l-1; 
                        break
                    else
                        l_max = l_min;
                        break
                    end
                end
            end
            % Add a check for empty voxels
            if length(Vij) == 0
            % elseif l_max - l_min < min_pts
            else
                r_min = Vij(l_min);
                r_max = Vij(l_max);
                below_rr = find(ptCloud_pol(:,2)<r_min & az_lower<rad2deg(ptCloud_pol(:,1)) & rad2deg(ptCloud_pol(:,1))<az_upper & elev_lower<rad2deg(ptCloud_pol(:,3)) & rad2deg(ptCloud_pol(:,3))<elev_upper);
                % ptCloud_new(below_rr,:) = NaN;
                above_rr = find(ptCloud_pol(:,2)>r_max & az_lower<rad2deg(ptCloud_pol(:,1)) & rad2deg(ptCloud_pol(:,1))<az_upper & elev_lower<rad2deg(ptCloud_pol(:,3)) & rad2deg(ptCloud_pol(:,3))<elev_upper);
                % ptCloud_new(above_rr,:) = NaN;
                points_vox_disc = cat(1, below_rr, above_rr);
                points_disc = cat(1, points_vox_disc, points_disc);
            end
        end
    end
end

% Function that implements the THIRD VERSION of the shadow mitigation algorithm
% Uses both clouds to find group of points that meets the threshold in both
% Inputs: 
% 1) Polar point cloud 1
% 2) Polar point cloud 2
% 3) Array of voxels 1 with distributed pts
% 4) Array of voxels 2 with distributed pts
% 5) Jump param
% 6) Min pts param
% 7) Arc length per voxel (az)
% 8) Band amount (elev)
% 9) Difference in elevation
% 10) Zero elevation point 
% Outputs: Array of points to be removed for each cloud

function [points_disc_1 points_disc_2] = shadow_mitig_3(ptCloud_pol_1, ptCloud_pol_2, Voxels1, Voxels2, jump, min_pts, arc_d, band_amt, elev_dif, zero_el)
    points_disc_1 = [];
    points_disc_2 = [];
    V_rad_1 = zeros(length(ptCloud_pol_1), 1);
    v_az = ceil(360/arc_d);
    v_elev = ceil(elev_dif/band_amt);
    for i = 1:v_az
        for j = 1:v_elev
            az_upper = (i*arc_d)-180;
            az_lower = (i*arc_d)-180 - arc_d;
            elev_upper = (j*band_amt)-zero_el;
            elev_lower = (j*band_amt)-zero_el-band_amt;
            Voxelij_1 = Voxels1{i,j};
            Voxelij_2 = Voxels2{i,j};
            radii_1 = ptCloud_pol_1(Voxelij_1,2);
            radii_2 = ptCloud_pol_2(Voxelij_2,2);
            Vij_1 = sort(radii_1);
            Vij_2 = sort(radii_2);
            l_min = 1;
            l_max = length(Vij_1);
            for l = 2:length(Vij_1)
                w = 0;
                diff = Vij_1(l) - Vij_1(l-1);
                if diff > jump
                    for t = l_min:l
                        range_jump1 = range_jump1 + Vij_1(t);
                    end
                    w = w+1;
                    jumps1(w,:) = range_jump1;
                    l_min = l;
                end
            end
            
            for l = 2:length(Vij_2)
                w = 0;
                diff = Vij_2(l) - Vij_2(l-1);
                if diff > jump
                    for t = l_min:l
                        range_jump2 = range_jump2 + Vij_2(t);
                    end
                    w = w+1;
                    jumps2(w,:) = range_jump2;
                    l_min = l;
                end
            end


            % NEXT STEPS:
            % loop through the jumps variables 
            % for each element, if the length of the row for each is over
            % the threshold, than that becomes the range of points we want



            %         if l - l_min > min_pts
            %             l_max = l-1; 
            %             break
            %         else
            %             l_min = l;
            % 
            %         end
            % end
            % % Add a check for empty voxels
            % if length(Vij_1) == 0
            % elseif l_max - l_min < min_pts
            % else
            %     r_min = Vij_1(l_min);
            %     r_max = Vij_1(l_max);
            %     below_rr = find(ptCloud_pol_1(:,2)<r_min & az_lower<rad2deg(ptCloud_pol_1(:,1)) & rad2deg(ptCloud_pol_1(:,1))<az_upper & elev_lower<rad2deg(ptCloud_pol_1(:,3)) & rad2deg(ptCloud_pol_1(:,3))<elev_upper);
            %     % ptCloud_new(below_rr,:) = NaN;
            %     above_rr = find(ptCloud_pol_1(:,2)>r_max & az_lower<rad2deg(ptCloud_pol_1(:,1)) & rad2deg(ptCloud_pol_1(:,1))<az_upper & elev_lower<rad2deg(ptCloud_pol_1(:,3)) & rad2deg(ptCloud_pol_1(:,3))<elev_upper);
            %     % ptCloud_new(above_rr,:) = NaN;
            %     points_vox_disc = cat(1, below_rr, above_rr);
            %     points_disc = cat(1, points_vox_disc, points_disc);
            % end
        end
    end
end

% Function that finds the mean point distribution in each CARTESIAN voxel
% Inputs:
% 1) Voxels - distributed
% 2) Original point cloud of the voxels 
% 3) Voxels in the Length 
% 4) Voxels in the width 
% 5) Voxels in the height
% Outputs: Voxels with mean point in each voxel 
function voxels_m = calc_means_cart(Voxels, ptCloud, v_L, v_W, v_H)
    voxels_m{v_L, v_W, v_H} = [];
    for i = 1:v_L
        for j = 1:v_W
            for k = 1:v_H
                x_m = ptCloud(Voxels{i,j,k},1);
                y_m = ptCloud(Voxels{i,j,k},2);
                z_m = ptCloud(Voxels{i,j,k},3);
                % Check if number is a NAN
                if isnan(x_m) | isnan(y_m) | isnan(z_m)
                else
                    mean_pt = [mean(x_m), mean(y_m), mean(z_m)];
                    voxels_m{i,j,k} = mean_pt;
                end
            end
        end
    end
end


% Function that finds the mean point distribution in each SPHERICAL voxel
% Inputs: 
% 1) Voxels - distributed
% 2) Original point cloud of the voxels
% 3) Voxels in the azimuth 
% 4) Voxels in the elevation range
% Outputs: Voxels with mean point in each voxel
function voxels_m = calc_means_sph(Voxels, ptCloud, v_az, v_elev)
    voxels_m{v_az,v_elev} = [];
    for i = 1:v_az
        for j = 1:v_elev
            x_m = ptCloud(Voxels{i,j},1);
            y_m = ptCloud(Voxels{i,j},2);
            z_m = ptCloud(Voxels{i,j},3);
            % Check if number is a NAN
            if isnan(x_m) | isnan(y_m) | isnan(z_m)
            else
                mean_pt = [mean(x_m), mean(y_m), mean(z_m)];
                voxels_m{i,j} = mean_pt;
            end
        end
    end
end

% Function that finds points containing the vehicle in a specific pt cloud
% Inputs: 
% 1) Original point cloud of the voxels
% 2) Polar point cloud of the voxels
% 3) Maximum range of the vehicle
function points_vehicle_rem = rem_vehicle(ptCloud, ptCloud_pol, range)
    points_vehicle_rem = [];
    for i = 1:length(ptCloud)
        if ptCloud_pol(i,2) < range
            points_vehicle_rem = [points_vehicle_rem i];
        end
    end
end


% Function that finds difference of means for CARTESIAN point clouds
% Inputs:
% 1) Means of voxels 1
% 2) Means of voxels 2
% 3) Voxel distribution 1
% 4) Voxel distribution 2
% 5) Voxels in the length
% 6) Voxels in the width
% 7) Voxels in the height
% Outputs: Voxels with mean difference between each voxel 
function means_dif = calc_mean_dif_cart(means1, means2, voxels1, voxels2, v_L, v_W, v_H)
    means_dif{v_L, v_W, v_H} = [];
    for i = 1:v_L
        for j = 1:v_W
            for k = 1:v_H
                % Specify a minimum number of points to recognize a voxel 
                if length(voxels1{i,j,k}) >= 50 && length(voxels2{i,j,k}) >= 50
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
end

% Function that finds difference of means for SPHERICAL point clouds 
% Inputs:
% 1) Means of voxels 1
% 2) Means of voxels 2
% 3) Voxel distribution 1
% 4) Voxel distribution 2
% 5) Voxels in the azimuth
% 6) Voxels in the elevation range
% 7) Minimum points threshold 
% Outputs: Voxels with mean difference bewteen each voxel 


function means_dif = calc_mean_dif_sph(means1, means2, voxels1, voxels2, v_az, v_elev, min_pts)
    means_dif{v_az, v_elev} = [];
    for i = 1:v_az
        for j = 1:v_elev
            % Specify a minimum number of points to recognize a voxel 
            if length(voxels1{i,j}) >= min_pts && length(voxels2{i,j}) >= min_pts
                x_dif = means2{i,j}(1,1) - means1{i,j}(1,1);
                y_dif = means2{i,j}(1,2) - means1{i,j}(1,2);
                z_dif = means2{i,j}(1,3) - means1{i,j}(1,3);
                if abs(x_dif) > 13 | abs(y_dif) > 13 | abs(z_dif) > 13
                    x_dif = 0;
                    y_dif = 0;
                    z_dif = 0;
                    mean_dif_pt = [x_dif y_dif z_dif];
                    means_dif{i,j} = mean_dif_pt;
                else
                    mean_dif_pt = [x_dif y_dif z_dif];
                    means_dif{i,j} = mean_dif_pt;
                end
            else
                means_dif{i,j} = [0 0 0];
            end
        end
    end
end

% Function that manufactures a quiver plot setup for CARTESIAN COORDS
% Inputs:
% 1) Number of total voxels 
% 2) Mean distribution within each voxel
% 3) Voxels in length
% 4) Voxels in width 
% 5) Voxels in height
% 6) X,Y,Z position desired
% Outputs: matrix with the values for the specified position and length

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

% Function that manufactures a quiver plot setup for SPHERICAL COORDS
% Inputs: 
% 1) Number of total voxels 
% 2) Mean distribtuion within each voxel 
% 3) Voxels in the azimuth
% 4) Voxels in the elevation range
% 5) X,Y,Z position desired
% Outputs: matrix with the values for the specified position and length

function quiv_i = quiver_setup_sph(vox_number, means, v_az, v_elev, pos)
    quiv_i = zeros(vox_number, 1);
    num = 1;
    for i = 1:v_az
        for j = 1:v_elev
            position_ii = means{i,j}(1,pos);
            quiv_i(num) = position_ii;
            num = num + 1;
        end
    end
end


% Function that locates the points causing domain boundary errors in a point cloud
% Inputs:  
% 1) Polar PtCloud
% 2) Height of the Lidar system
% 3) Angle specification
% Outputs: points to be removed to eliminate domain boundary errors 

function points_rem = domain_error_below(ptCloud_pol, height, angle)
    points_rem = [];
    % rad_bound = abs(height/tan(deg2rad(angle)));
    degree_bound = deg2rad(angle);
    for i = 1:length(ptCloud_pol)
        if ptCloud_pol(i,3) < degree_bound
            points_rem = [points_rem i];
        else
        end
    end
end

% Function that locates the points causing domain boundary errors in a point cloud
% Inputs:  
% 1) Polar ptCloud
% 2) Height of the Lidar system
% 3) Angle specification
% Outputs: points to be removed to eliminate domain boundary errors 

function points_rem = domain_error_above(ptCloud_pol, height, angle)
    points_rem = [];
    % rad_bound = abs(height/tan(deg2rad(angle)));
    degree_bound = deg2rad(angle);
    for i = 1:length(ptCloud_pol)
        if ptCloud_pol(i,3) > degree_bound
            points_rem = [points_rem i];
        else
        end
    end
end