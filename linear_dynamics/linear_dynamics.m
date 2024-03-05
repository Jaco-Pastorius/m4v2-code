clear all
close all
clc

m = 5.0;
g = 9.81;
J = diag([0.002,0.008,0.007]);
Jinv = inv(J);
w = 0.0775;
d = 0.1325;
l = 0.16;

syms z dz phi th psi ox oy oz f1 f2 f3 f4 varphi
x = [z;dz;phi;th;psi;ox;oy;oz];
u = [f1;f2;f3;f4];

f = f(x, u, w, d, l, varphi, m, g, J, Jinv);
A = matlabFunction(simplify(jacobian(f,x)),"File",'A.m');
B = matlabFunction(simplify(jacobian(f,u)),"File",'B.m');

