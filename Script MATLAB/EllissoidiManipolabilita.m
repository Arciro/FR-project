%% Elaborato di Fondamenti di Robotica
% Autori: Ciro Arena P38000053 e Vito Giura P38000056
% Calcolo degli ellissoidi di manipolabilità

close;
clear;
clc;

%% Dati del problema

d0 = 1;
a1 = 0.5;
a2 = 0.5;
d4 = 0.2;

%% Convenzione di Denavit-Hartenberg

L1 = Link('d',0,'a',a1,'alpha',0);
%L1 = Revolute('a', a1);
L2 = Link('d',0,'a',a2,'alpha',pi);
%L2 = Revolute('a', a2, 'alpha', pi);
L3 = Link('theta',0,'a',0,'alpha',0);
%L3 = Prismatic();
L3.qlim = [0 1];

%% Cinematica diretta

%Creazione del manipolatore
scara_sp = SerialLink([L1 L2 L3], 'name', 'SCARA');
scara_sp.base = SE3(0, 0, d0);


%% Ellissoidi di manipolabilità in forza e velocità

theta2 = -180;

%ellissoide di velocità
subplot(1,2,1)
scara_sp.vellipse([0 theta2 0], 'shadow');
title('Ellissoidi di velocità')
xlabel('Asse x')
ylabel('Asse y')
zlabel('Asse z')
axis tight
axis equal
grid on

%scara_sp.teach('callback', @(r,q) r.vellipse(q))

%ellissoide di forza
subplot(1,2,2)
scara_sp.fellipse([0 theta2 0], 'shadow');
title('Ellissoidi di forza')
xlabel('Asse x')
ylabel('Asse y')
zlabel('Asse z')
axis tight
axis equal
%scara_sp.teach('callback', @(r,q) r.fellipse(q))
grid on
