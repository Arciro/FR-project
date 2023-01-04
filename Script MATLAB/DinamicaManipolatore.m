%% Elaborato di Fondamenti di Robotica
% Autori: Ciro Arena P38000053 e Vito Giura P38000056
% Dinamica del manipolatore

clc
clear
close

%% Dati del problema

d0 = 1;
a1 = 0.5;
a2 = 0.5;
d4 = 0.2;

% variabili di giunto
syms theta1 theta2 d3 theta4 a1 a2 d4;
q=[theta1 theta2 d3 theta4];

%% Convenzione di Denavit-Hartenberg

L1 = Link('d',0,'a',a1,'alpha',0);
L1 = L1.sym();
%L1 = Revolute('a', a1);
L2 = Link('d',0,'a',a2,'alpha',pi);
L2 = L2.sym();
%L2 = Revolute('a', a2, 'alpha', pi);
L3 = Link('theta',0,'a',0,'alpha',0);
L3 = L3.sym();
%L3 = Prismatic();
L3.qlim = [0 1];
L4 = Link('d',d4,'a',0,'alpha',0);
L4 = L4.sym();
%L4 = Revolute('d', d4);

%% Cinematica diretta

%Creazione del manipolatore
scara = SerialLink([L1 L2 L3 L4], 'name', 'SCARA'); 

scara = scara.sym();
A01 = L1.A(theta1);
A12 = L2.A(theta2);
A23 = L3.A(d3);
A34 = L4.A(theta4);

scara.base = SE3(0, 0, d0);
Tb0 = scara.base;
Teb = scara.fkine(q);

%% Dinamica

syms ml1 ml2 ml3 ml4 load L1 L2 Il1 Il2 Il4 Kr1 Kr2 Kr3 Kr4 Im1 Im2 Im3 Im4 Fm1 Fm2 Fm3 Fm4;

%masse
scara.links(1).m = ml1;
scara.links(2).m = ml2;
scara.links(3).m = ml3;
%scara.links(4).m = ml4;

%posizione COM
scara.links(1).r = [L1-a1, 0, 0];
scara.links(2).r = [L2-a2, 0, 0];

%inerzie dei bracci
scara.links(1).I = [0 0 0; 0 0 0; 0 0 Il1+Im2];
scara.links(2).I = [0 0 0; 0 0 0; 0 0 Il2+Im3];
scara.links(4).I = [0 0 0; 0 0 0; 0 0 Il4+Im4];

%rapporti di trasmissione
scara.links(1).G = Kr1;
scara.links(2).G = Kr2;
scara.links(3).G = Kr3;
scara.links(4).G = Kr4;

%inerzie dei motori
scara.links(1).Jm = Im1;
scara.links(2).Jm = Im2;
scara.links(3).Jm = Im3;
scara.links(4).Jm = Im4;

%attriti dei motori
scara.links(1).B = Fm1;
scara.links(2).B = Fm2;
scara.links(3).B = Fm3;
scara.links(4).B = Fm4;

scara.payload(load,[0 0 0.1]);
syms q1 q2 q3 q4 q1d q2d q3d q4d q1dr q2dr q3dr q4dr q1dd q2dd q3dd q4dd q1ddr q2ddr q3ddr q4ddr;
q = [q1 q2 q3 q4];
qd = [q1d q2d q3d q4d];
qdr = [q1dr q2dr q3dr q4dr];
qdd = [q1dd q2dd q3dd q4dd];
qddr = [q1ddr q2ddr q3ddr q4ddr];

B = scara.inertia(q);
B = simplify(B);
C = scara.coriolis(q, qd);
C = simplify(C);
Fv = scara.friction(qd);
Fv = simplify(Fv);
g = scara.gravload(q);
g = simplify(g);
tau = scara.rne(q,qd,qdd);
tau = simplify(tau);

Fvr = scara.friction(qdr);
Fvr = simplify(Fvr);
tau_r = B*qddr.'+C*qdr.'+Fvr.'+g.';
tau_r = expand(tau_r);

%% REGRESSORE

Y = sym(zeros(4,14));
Y(1,1) = q1ddr*L1^2;
Y(1,2) = q1ddr;
Y(1,3) = q1ddr*Kr1^2;
Y(1,4) = -q1dr*Kr1^2;
Y(1,5) = (q1ddr+q2ddr)*L2^2+q1ddr*a1^2+ 2*L2*a1*q1ddr*cos(q2) + L2*a1*q2ddr*cos(q2) - L2*a1*q1d*q2dr*sin(q2) - L2*a1*q2d*q1dr*sin(q2) - L2*a1*q2d*q2dr*sin(q2);
Y(1,6) = q1ddr+q2ddr;
Y(1,7) = q1ddr;
Y(1,8) = 0;
Y(1,9) = (a1^2+a2^2)*q1ddr + q2ddr*a2^2 + 2*a1*a2*q1ddr*cos(q2) + a1*a2*q2ddr*cos(q2)-a1*a2*q1d*q2dr*sin(q2)-a1*a2*q2d*q1dr*sin(q2)-a1*a2*q2d*q2dr*sin(q2);
Y(1,10) = q1ddr+q2ddr;
Y(1,11) = 0;
Y(1,12) = q1ddr+q2ddr-q4ddr;
Y(1,13) = q1ddr+q2ddr-q4ddr;
Y(1,14) = 0;

Y(2,1) = 0;
Y(2,2) = 0;
Y(2,3) = 0;
Y(2,4) = 0;
Y(2,5) = (q1ddr+q2ddr)*L2^2 + L2*a1*q1ddr*cos(q2) + L2*a1*q1d*q1dr*sin(q2);
Y(2,6) = q1ddr+q2ddr;
Y(2,7) = q2ddr*Kr2^2;
Y(2,8) = -q2dr*Kr2^2;
Y(2,9) = (q2ddr+q1ddr)*a2^2 + a1*a2*q1ddr*cos(q2) + a1*a2*q1d*q1dr*sin(q2);
Y(2,10) = q1ddr+q2ddr;
Y(2,11) = 0;
Y(2,12) = q1ddr+q2ddr-q4ddr;
Y(2,13) = q1ddr+q2ddr-q4ddr;
Y(2,14) = 0;

Y(3,1) = 0;
Y(3,2) = 0;
Y(3,3) = 0;
Y(3,4) = 0;
Y(3,5) = 0;
Y(3,6) = 0;
Y(3,7) = 0;
Y(3,8) = 0;
Y(3,9) = q3ddr-9.81;
Y(3,10) = q3ddr*Kr3^2;
Y(3,11) = -q3dr*Kr3^2;
Y(3,12) = 0;
Y(3,13) = 0;
Y(3,14) = 0;

Y(4,1) = 0;
Y(4,2) = 0;
Y(4,3) = 0;
Y(4,4) = 0;
Y(4,5) = 0;
Y(4,6) = 0;
Y(4,7) = 0;
Y(4,8) = 0;
Y(4,9) = 0;
Y(4,10) = 0;
Y(4,11) = 0;
Y(4,12) = q4ddr-q2ddr-q1ddr;
Y(4,13) = q4ddr-q2ddr-q1ddr+q4ddr*Kr4^2;
Y(4,14) = -q4dr*Kr4^2;

P = [ml1; Il1; Im1; Fm1; ml2; Il2; Im2; Fm2; (ml3+load); Im3; Fm3; Il4; Im4; Fm4];