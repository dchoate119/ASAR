clc 
clear all 

load('wind','x','y','z','u','v','w');
X = x(5:10,20:25,6:10); 
Y = y(5:10,20:25,6:10); 
Z = z(5:10,20:25,6:10); 
U = u(5:10,20:25,6:10); 
V = v(5:10,20:25,6:10); 
W = w(5:10,20:25,6:10);

quiver3(X,Y,Z,U,V,W)
axis equal
