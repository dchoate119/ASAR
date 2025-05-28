% Exploring error bias 
% Seeing if matlab can invert my large matrix 
clear all 
syms s t p1 q1 p2 q2 d

% Create 4x4 J matrix 
% Wedge 1
% Note: wedge locations = p1 q1 p2 q2
% j11 = cos(t)*p1 - sin(t)*q1;
% j12 = -s*(sin(t)*p1 + cos(t)*q1);
% j21 = sin(t)*p1 + cos(t)*q1;
% j22 = s*(cos(t)*p1 - sin(t)*q1);
% % Note: wedge locations = 0 0 d 0
% j11 = 0;
% j12 = 0;
% j21 = 0;
% j22 = 0;
% Note: wedge locations = -d/2 0 d/2 0
j11 = cos(t)*(-d/2);
j12 = -s*(sin(t)*(-d/2));
j21 = sin(t)*(-d/2);
j22 = s*(cos(t)*(-d/2));

% Wedge 2
% Note: wedge locations = p1 q1 p2 q2
% j31 = cos(t)*p2 - sin(t)*q2;
% j32 = -s*(sin(t)*p2 + cos(t)*q2);
% j41 = sin(t)*p2 + cos(t)*q2;
% j42 = s*(cos(t)*p2 - sin(t)*q2);
% % Note: wedge locations = 0 0 d 0
% j31 = cos(t)*d;
% j32 = -s*sin(t)*d;
% j41 = sin(t)*d;
% j42 = s*cos(t)*d;
% Note: wedge locations = -d/2 0 d/2 0 
j31 = cos(t)*(d/2);
j32 = -s*sin(t)*(d/2);
j41 = sin(t)*(d/2);
j42 = s*cos(t)*(d/2);

% Constants
j13 = 1;
j14 = 0;
j23 = 0;
j24 = 1;
j33 = 1;
j34 = 0;
j43 = 0;
j44 = 1;

% Combine 
J = [j11 j12 j13 j14; j21 j22 j23 j24;
    j31 j32 j33 j34; j41 j42 j43 j44];

% disp(J)

% % Transpose
% J_T = transpose(J);
% % disp("Transpose")
% % disp(J_T)
% % J_T * J
% jtj = J_T * J;
% % disp("JTJ")
% % disp(JTJ)
% JTJ = simplify(jtj);
% % disp("Simplified JTJ")
% % disp(jtj)
% % Inverse
% jtj_i = inv(JTJ);
% JTJ_i = simplify(jtj_i);
% disp("Simplified JTJ^-1")
% disp(JTJ_i)

% JUST inverse of J 
ji = inv(J);
Ji = simplify(ji);
disp("Simplfied J^-1")
disp(Ji)


% Delta Y
syms x1 y1 x2 y2
Dy = [x1;y1;x2;y2];

% Using J^-1
dalpha_ji = Ji*Dy;
Dalpha_ji = simplify(dalpha_ji);

% % Using jtjijt
% dalpha_jtji = JTJ_i*J_T*Dy;
% Dalpha_jtji = simplify(dalpha_jtji);

disp("Dalpha using J^-1")
disp(Dalpha_ji)
% disp("Dalpha using JTJ^-1")
% disp(Dalpha_jtji)