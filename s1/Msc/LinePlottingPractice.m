% Plotting practice 
clear all 
close all 


t = [0, 100];
x = sin(t);
y = cos(t);
z = t;

plot3(x, y, z)

hold on 
plot3(1 + cosd(-160)*[0, 10], 1 + sind(-160)*[0,10], [0,10])

% Define edge endpoints in polar coordinates

rho = [1; 2];

Az = [10; 10]*pi/180;

El = [-10; -10]*pi/180;

% Convert endpoints to Cartesian coordinates & plot

X = rho.*cos(Az).*cos(El);

Y = rho.*sin(Az).*cos(El);

Z = rho.*sin(El);

plot3(X,Y,Z)