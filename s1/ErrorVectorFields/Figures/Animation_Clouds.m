% Daniel Choate 
% Creating an animation to better display vector field plot 
% Data generated from LidarErrorAnalysis_ExpD

clear all 
close all 

% Load in data for original vector field plot
% Loading 6 variables X, Y, Z, U, V, W
load vPlotOrig

figure(1)
quiver3(X_q, Y_q, Z_q, U_q, V_q, W_q, 'AutoScale', 'off')

% view(0,90)
% view(45,45)
% view(90,0)
% view(45,45)
% view(0,90)
% view(45,45)
% view(90,0)

% Set variable for degrees per frame 
d = 1;
t = 90/d;
s1 = 0; % Starting rotation for the azimuth 
s2 = 90; % Starting rotation for the elevation 

% set(gca,"NextPlot","replacechildren")
v = VideoWriter("VectorFieldEX.avi");%,"Indexed AVI");
open(v)
% v.colormap = gcf;

% Loop to gather frames between desired angles from a to b
f1 = figure(1);
% f1.Visible = 'off';
fs = [];
for i = 1:t+1
    view(s1,s2)
    f = getframe(gcf);
    writeVideo(v,f)
    fs = [fs f];
    s1 = s1+d;
    s2 = s2-d;
end


% Loop to gather frames between desired angles from b to a
s1 = 90;
s2 = 0;
figure(1)
for i = 1:t+1
    view(s1,s2)
    f = getframe(gcf);
    writeVideo(v,f)
    fs = [fs f];
    s1 = s1-d;
    s2 = s2+d;
end

axis('off')
axis tight 
% ax = gca;
% ax.NextPlot = 'replaceChildren';
h = figure;
% h.Visible = 'off';
movie(h,fs,2);


close(v)