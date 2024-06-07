% Vector field visualization example 
% Creating an example vector field for figure paper 

clear all
close all

% Randomize two sets of points 
L = 1000;
y1 = randi([1,100],L,1);
y2 = randi([1,100],L,1);

% Randomize two sets of points for scenario 3

y1 = randi([1,100],L,1);
for i = 1:L
    num = y1(i,1);
    if y1(i,1) > 10 && y1(i,1) < 33
        y1(i,1) = randi([33,66],1,1);
    end
    if y1(i,1) > 66 && y1(i,1) < 90
        y1(i,1) = randi([33,66],1,1);
    end
end

y2 = randi([1,100],L,1);

step = 100/L;

x1 = (step:step:100);
x1 = transpose(x1);

x2 = (step:step:100);
x2 = transpose(x2);

set1 = [x1,y1];
set2 = [x2,y2];

% plot(x1, y1, '.', 'MarkerSize', 3)
% hold on 
% plot(x2, y2, '.', 'MarkerSize', 3)

voxels1{3,3} = [];
voxels2{3,3} = [];

v_side = 100/3;

% Set 1 into voxels
for i = 1:length(y1)
    x = set1(i,1);
    y = set1(i,2);
    xvoxInd = ceil(x/v_side);
    yvoxInd = ceil(y/v_side);
    if xvoxInd < 1; xvoxInd = 1; end
    if yvoxInd < 1; yvoxInd = 1; end
    voxels1{xvoxInd, yvoxInd} = [voxels1{xvoxInd, yvoxInd}; set1(i,:)];
end


% Set 2 into voxels 
for i = 1:length(y2)
    x = set2(i,1);
    y = set2(i,2);
    xvoxInd = ceil(x/v_side);
    yvoxInd = ceil(y/v_side);
    if xvoxInd < 1; xvoxInd = 1; end
    if yvoxInd < 1; yvoxInd = 1; end
    voxels2{xvoxInd, yvoxInd} = [voxels2{xvoxInd, yvoxInd}; set2(i,:)];
end

% Calculate means in each voxel 
means1{3,3} = [];
means2{3,3} = [];

for i = 1:3
    for j = 1:3
        xmean1 = mean(voxels1{i,j}(:,1));
        ymean1 = mean(voxels1{i,j}(:,2));
        xmean2 = mean(voxels2{i,j}(:,1));
        ymean2 = mean(voxels2{i,j}(:,2));
        means1{i,j} = [xmean1 ymean1];
        means2{i,j} = [xmean2 ymean2];
    end
end

% Calculate mean difference 
means_dif{3,3} = [];
for i = 1:3
    for j = 1:3
        x_dif = means2{i,j}(1,1) - means1{i,j}(1,1);
        y_dif = means2{i,j}(1,2) - means1{i,j}(1,2);
        means_dif{i,j} = [x_dif y_dif];
    end
end

X = [];
Y = [];
U = [];
V = [];

vox_num = 9;
for i = 1:3
    for j = 1:3
        X = [X means1{i,j}(1,1)];
        Y = [Y means1{i,j}(1,2)];
        U = [U means_dif{i,j}(1,1)];
        V = [V means_dif{i,j}(1,2)];
    end
end


% Plot dashed lines to separate voxels 
xh = [33,33];
yh = [1,100];
plot(xh,yh, '--','color', 'black')

xh = [66,66];
yh = [1,100];
hold on
plot(xh,yh, '--','color', 'black')

xh = [1,100];
yh = [33,33];
plot(xh,yh, '--','color', 'black')

xh = [1,100];
yh = [66,66];
plot(xh,yh, '--','color', 'black')

% xh = [100,100];
% yh = [1,100];
% plot(xh,yh, '--','color', 'black')

% quiver(X,Y,U,V, 'color', 'black', 'AutoScale','off')


% Making quiver plot WITHOUT THE ACTUAL DATA 
% Easier to manipulate/much faster

% Position vectors x and y 
pos_13 = 100/6;
pos_23 = 50;
pos_33 = pos_13*5;
half_13 = pos_13/2;

% For first plot
XX = [pos_13 pos_23 pos_33 pos_13 pos_23 pos_33 pos_13 pos_23 pos_33];
YY = [pos_13/2 pos_13/2 pos_13/2 pos_23-half_13 pos_23-half_13 pos_23-half_13 pos_33-half_13 pos_33-half_13 pos_33-half_13];
dist = 15;
UU = [0 0 0 0 0 0 0 0 0];
VV = [dist dist dist dist dist dist dist dist dist];

% For second plot
XX2 = [pos_13 pos_23 pos_33 pos_13 pos_23 pos_33 pos_13 pos_23 pos_33];
YY2 = [pos_13/2 pos_13/2 pos_13/2 pos_23 pos_23 pos_23 pos_33+half_13 pos_33+half_13 pos_33+half_13];
UU2 = [0 0 0 1.5 -1.5 0 0 0 0];
VV2 = [dist dist dist -1.5 .5 1.5 -dist -dist -dist];

% For third plot 
XX3 = [pos_13 pos_23 pos_33 pos_13 pos_23 pos_33 pos_13 pos_23 pos_33];
YY3 = [pos_13 pos_13 pos_13 pos_23 pos_23 pos_23 pos_33 pos_33 pos_33];
Numbers_u = (2-(-2)).*rand(9,1)+(-2);
Numbers_v = (2-(-2)).*rand(9,1)+(-2);
UU3 = transpose(Numbers_u);
VV3 = transpose(Numbers_v);


hold on
quiver(XX3, YY3, UU3, VV3, 'Color', '#0072BD', 'AutoScale','off')
