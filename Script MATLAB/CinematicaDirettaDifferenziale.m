%% Elaborato di Fondamenti di Robotica
% Autori: Ciro Arena P38000053 e Vito Giura P38000056
% Calcolo della cinematica diretta e differenziale

close;
clear;
clc;

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

%% Cinematica differenziale

%calcolo Jacobiano geometrico
J = scara.jacob0(q);
J = simplify(J);

% vediamo come eliminare eventuali righe nulle
r=length(J);
count=0;
v=zeros(r,1);
for i=1:r
   if J(i,:)==zeros(r,1)
       v(i)=i;
   end
end
for i=1:r
   if v(i)~=0
      J(i-count,:)=[];
      count=count+1;
   end
end

%singolarità cinematiche
singKin=simplify(det(J));

%misura di manipolabilità
Jp = J(1:3, 1:3);
mis_manip = sqrt(simplify(det(Jp*Jp.')));