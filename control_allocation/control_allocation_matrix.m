clear all
close all
clc

syms phi cT kM0 kM1 kM2 kM3 lx ly real
syms n1 n2 n3 n4 real 
syms cz taux tauy tauz real
syms d2 d3 real

phi_data = [0,10,20,30,40,50,60,70,80,90];
thrust_data = [1,1,0.95,0.9,0.83,0.75,0.73,0.68,0.52,0.4];

% test angle
phi_test = deg2rad(88);

% automatically generate positions
p1 = [lx;d2+d3*cos(phi);d3*sin(phi)];
p2 = [-lx;-d2-d3*cos(phi);d3*sin(phi)];
p3 = [lx;-d2-d3*cos(phi);d3*sin(phi)];
p4 = [-lx;d2+d3*cos(phi);d3*sin(phi)];

% automatically generate axes
% Quadrotor H frame

initial_axis = [0,0,-1];

axis1 = [0,sin(phi),-cos(phi)];
axis2 = [0,-sin(phi),-cos(phi)];
axis3 = [0,-sin(phi),-cos(phi)];
axis4 = [0,sin(phi),-cos(phi)];

% px4 allocation
thrust1 = cT*axis1;
thrust2 = cT*axis2;
thrust3 = cT*axis3;
thrust4 = cT*axis4;

moment1 = cT * cross(p1,axis1) - cT * kM0 * axis1; % this has a crossover
moment2 = cT * cross(p2,axis2) - cT * kM1 * axis2;
moment3 = cT * cross(p3,axis3) - cT * kM2 * axis3; % this has a crossover
moment4 = cT * cross(p4,axis4) - cT * kM3 * axis4;

A_px4 = sym(zeros(4,6));
A_px4(1,:) = [moment1,thrust1];
A_px4(2,:) = [moment2,thrust2];
A_px4(3,:) = [moment3,thrust3];
A_px4(4,:) = [moment4,thrust4];
% A_px4 = A_px4';

% numerical evaluation

% CLOCKWISE ROTATION IN PX4 means positive kM

A_px4_num = vpa(subs(A_px4, [phi,cT,kM0,kM1,kM2,kM3,lx,ly,d2,d3],[phi_test,6.5,0.05,0.05,-0.05,-0.05,0.16,0.21,0.0775,0.1325]));

axis1_num = vpa(subs(axis1,phi,phi_test));
axis2_num = vpa(subs(axis2,phi,phi_test));
axis3_num = vpa(subs(axis3,phi,phi_test));
axis4_num = vpa(subs(axis4,phi,phi_test));

p1_num = vpa(subs(p1,[phi,lx,ly,d2,d3],[phi_test,0.16,0.21,0.0775,0.1325]));
p2_num = vpa(subs(p2,[phi,lx,ly,d2,d3],[phi_test,0.16,0.21,0.0775,0.1325]));
p3_num = vpa(subs(p3,[phi,lx,ly,d2,d3],[phi_test,0.16,0.21,0.0775,0.1325]));
p4_num = vpa(subs(p4,[phi,lx,ly,d2,d3],[phi_test,0.16,0.21,0.0775,0.1325]));



% % allocation matrix (my way)
% a = cT*cos(phi);
% b = ly*cT;
% c = lx*cT*cos(phi) - cM*sin(phi);
% d = cM*cos(phi) + lx*cT*sin(phi);
% 
% A = [-a,-a,-a,-a;
%      -b,b,b,-b;
%      c,-c,c,-c;
%      d,d,-d,-d ];
% 
% Ainv = simplify(inv(A));
% 
% % numerics
% (
% A_num = vpa(subs(A, [phi,cT,cM,lx,ly],[phi_test,6.5,0.05,0.16,0.21]));
% Ainv_num = vpa(subs(Ainv, [phi,cT,cM,lx,ly],[phi_test,6.5,0.05,0.16,0.21]));


