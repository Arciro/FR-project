%% Elaborato di Fondamenti di Robotica
% Autori: Ciro Arena P38000053 e Vito Giura P38000056
% Generazione traiettoria e inversione cinematica

clear
clc
close

Tc = 0.001; %tempo di campionamento
%% Punti della traiettoria

punto = [0.35 0.8 0.4];
head_points = [
    punto(1)        punto(2)        punto(3); %punto 1
    punto(1)-0.7    punto(2)        punto(3); %punto 2
    -0.55           punto(2)-0.1    punto(3); %punto 3
    -0.65           punto(2)-0.25   punto(3); %punto 4
    -0.7            punto(2)-0.4    punto(3); %punto 5
    -0.7            punto(2)-1.2    punto(3); %punto 6
    -0.65           0.25-punto(2)   punto(3); %punto 7
    -0.55           0.1-punto(2)    punto(3); %punto 8
    punto(1)-0.7    -punto(2)       punto(3); %punto 9
    punto(1)        -punto(2)       punto(3); %punto 10
    0.55            0.1-punto(2)    punto(3); %punto 11
    0.65            0.25-punto(2)   punto(3); %punto 12
    0.7             punto(2)-1.2    punto(3); %punto 13
    0.7             punto(2)-0.4    punto(3); %punto 14
    0.65            punto(2)-0.25   punto(3); %punto 15
    0.55            punto(2)-0.1    punto(3); %punto 16
    punto(1)        punto(2)        punto(3); %punto 17
    ];

head_points=head_points';              

head_to_eye_point = [0.4; 0.55; punto(3)+0.2]; %punto 18

eyes_points=[ 
     0.4        0.35    punto(3); %punto cir.sx 19
     0.4        0.15    punto(3)  %centro cir.sx 20
     0          0.35    punto(3)+0.2;
     -0.4       0.35    punto(3); %punto cir.sx 19
     -0.4       0.15    punto(3)  %centro cir.sx 20
 ];

eyes_points = eyes_points';

eye_to_ear_point = [-0.55; 0.35; punto(3)+0.2];

% 
% nose_points=[
%     -0.025                  0.75                       0; %punto 19
%     0.025                   0.75                       0  %punto 20
% ];
% nose_points=nose_points';
% 
% mouth_points=[
%     -0.06                   0.65                       0; % boccsupsx punto 21
%      0.06                   0.65                       0; %bocca supdx punto 22
%      0.04                   0.625                      0; %bocca infdx punto 23
%     -0.04                   0.625                      0  %bocca infsx punto 24
% ];
% mouth_points=mouth_points';
% 
ears_points=[
    -0.7    0.35    punto(3)
];
ears_points = ears_points';

%head_times = [0 4 10 14 20 24 30 34 40];
head_times = [0 2 6.5 8.5 13 15 19.5 21.5 26];
head_to_eye_time = [0 3];
%eyes_times = [0 4 10 14];
eyes_times = [0 4.5 7.5 12]; 
eye_to_ear_time = [0 3];
% nose_times = [44 49];
% mouth_times= [49 52 55 58 61];
ears_times = [0 4];
% 
 %% TESTA

% p1 percorso rettilineo
t1=0:Tc:head_times(2)-head_times(1);
T1=transl(head_points(1,1),head_points(2,1),head_points(3,1));
T2=transl(head_points(1,2),head_points(2,2),head_points(3,2));
[s,sd,sdd]=lspb(0,1,t1);
p1=ctraj(T1,T2,s);
p1=transl(p1);
p1d = derivata_prima(p1,Tc);
p1dd = derivata_seconda(p1,Tc);

% p2 punti di via
t2 = (head_times(3)-head_times(2))/3;
p2 = mstraj(head_points(:,[3 4 5])',[],[t2,t2,t2], head_points(:,2)', Tc, 1);
p2=[head_points(:,2)'; p2];
p2=p2(1:end-1,:);
p2d = derivata_prima(p2,Tc);
p2dd = derivata_seconda(p2,Tc);

% p3 percorso rettilineo
t3=0:Tc:head_times(4)-head_times(3);
T5=transl(head_points(1,5),head_points(2,5),head_points(3,5));
T6=transl(head_points(1,6),head_points(2,6),head_points(3,6));
[s,sd,sdd]=lspb(0,1,t3);
p3=ctraj(T5,T6,s);
p3=transl(p3);
p3=p3(1:end-1,:);
p3d = derivata_prima(p3,Tc);
p3dd = derivata_seconda(p3,Tc);

% p4 punti di via
t4 = (head_times(5)-head_times(4))/3;
p4 = mstraj(head_points(:,[7 8 9])',[],[t4,t4,t4],head_points(:,6)',Tc, 1);
p4=[head_points(:,6)'; p4];
p4=p4(1:end-1,:);
p4d = derivata_prima(p4,Tc);
p4dd = derivata_seconda(p4,Tc);

% p5 percorso rettilineo

t5=0:Tc:head_times(6)-head_times(5);
T9=transl(head_points(1,9),head_points(2,9),head_points(3,9));
T10=transl(head_points(1,10),head_points(2,10),head_points(3,10));
[s,sd,sdd]=lspb(0,1,t5);
p5=ctraj(T9,T10,s);
p5=transl(p5);
p5=p5(1:end-1,:);
p5d = derivata_prima(p5,Tc);
p5dd = derivata_seconda(p5,Tc);

% p6 punti di via
t6 = (head_times(7)-head_times(6))/3;
p6 = mstraj(head_points(:,[11 12 13])',[],[t6,t6,t6],head_points(:,10)',Tc, 1);
p6=[head_points(:,10)'; p6];
p6=p6(1:end-1,:);
p6d = derivata_prima(p6,Tc);
p6dd = derivata_seconda(p6,Tc);

% p7 percorso rettilineo
t7=0:Tc:head_times(8)-head_times(7);
T13=transl(head_points(1,13),head_points(2,13),head_points(3,13));
T14=transl(head_points(1,14),head_points(2,14),head_points(3,14));
[s,sd,sdd]=lspb(0,1,t7);
p7=ctraj(T13,T14,s);
p7=transl(p7);
p7=p7(1:end-1,:);
p7d = derivata_prima(p7,Tc);
p7dd = derivata_seconda(p7,Tc);

% p8 punti di via
t8 = (head_times(9)-head_times(8))/3;
p8 = mstraj(head_points(:,[15 16 17])',[],[t8,t8,t8],head_points(:,14)',Tc, 1);
p8=[head_points(:,14)'; p8];
p8=p8(1:end-1,:);
p8d = derivata_prima(p8,Tc);
p8dd = derivata_seconda(p8,Tc);

%% OCCHI
 
% p9 punti di via
t9 = (head_to_eye_time(2)-head_to_eye_time(1))/2;
wp1 = [head_points(:,17)'; head_to_eye_point'; eyes_points(:,1)']';
p9 = mstraj(wp1(:,[2 3])',[],[t9,t9],wp1(:,1)',Tc, 1);
p9=[head_points(:,17)'; p9];
p9=p9(1:end-1,:);
p9d = derivata_prima(p9,Tc);
p9dd = derivata_seconda(p9,Tc);
 
 % circonferenza sx ext punto 19
z = [0; 0; 1];
t10 = eyes_times(2)-eyes_times(1);
[p10,p10d,p10dd] = circonferenza(eyes_points(:,1),z,eyes_points(:,2),2*pi,0,t10,Tc,"lspb");
p10=p10(1:end-1,:);
p10d=p10d(1:end-1,:);
p10dd=p10dd(1:end-1,:);

% p9 punti di via
t11 = (eyes_times(3)-eyes_times(2))/2;
wp2 = [eyes_points(:,1)'; eyes_points(:,3)'; eyes_points(:,4)']';
p11 = mstraj(wp2(:,[2 3])',[],[t11,t11],wp2(:,1)',Tc, 1);
p11=[eyes_points(:,1)'; p11];
p11=p11(1:end-1,:);
p11d = derivata_prima(p11,Tc);
p11dd = derivata_seconda(p11,Tc);

% circonferenza dx punto 19
z = [0; 0; 1];
t12 = eyes_times(4)-eyes_times(3);
[p12,p12d,p12dd] = circonferenza(eyes_points(:,4),z,eyes_points(:,5),2*pi,0,t12,Tc,"lspb");
p12=p12(1:end-1,:);
p12d=p12d(1:end-1,:);
p12dd=p12dd(1:end-1,:);

%% ORECCHIE
% p9 punti di via
t13 = (eye_to_ear_time(2)-eye_to_ear_time(1))/2;
wp3 = [eyes_points(:,4)'; eye_to_ear_point'; ears_points(:,1)']';
p13 = mstraj(wp3(:,[2 3])',[],[t13,t13],wp3(:,1)',Tc, 1);
p13=[eyes_points(:,4)'; p13];
p13=p13(1:end-1,:);
p13d = derivata_prima(p13,Tc);
p13dd = derivata_seconda(p13,Tc);

%% plot
tempo_testa=0:Tc:head_times(end);
testa = [p1; p2; p3; p4; p5; p6; p7; p8];
testad = [p1d; p2d; p3d; p4d; p5d; p6d; p7d; p8d];
testadd = [p1dd; p2dd; p3dd; p4dd; p5dd; p6dd; p7dd; p8dd];

head_to_eye = p9;
head_to_eyed = p9d;
head_to_eyedd = p9dd;

occhi = [p10; p11; p12];
occhid = [p10d; p11d; p12d];
occhidd = [p10dd; p11dd; p12dd];

eye_to_ear = p13;
eye_to_eard = p13d;
eye_to_eardd = p13dd;


traj = [testa; head_to_eye; occhi; eye_to_ear];
trajd = [testad; head_to_eyed; occhid; eye_to_eard];
trajdd = [testadd; head_to_eyed; occhidd; eye_to_eardd];
figure(1)
plot3(traj(:,1),traj(:,2),traj(:,3), 'b', 'LineWidth', 2)
t=0:Tc:head_times(end)+head_to_eye_time(end)+eyes_times(end)+eye_to_ear_time(end);

% hold on
% occhio_sx = p9;
% plot3(occhio_sx(:,1),occhio_sx(:,2),occhio_sx(:,3), 'r', 'LineWidth', 2)
% hold on
% occhio_dx = p10;
% plot3(occhio_dx(:,1),occhio_dx(:,2),occhio_dx(:,3), 'r', 'LineWidth', 2)
% hold on
% naso = p11;
% plot3(naso(:,1),naso(:,2),naso(:,3))
% hold on
% bocca = [p12;p13;p14;p15];
% plot3(bocca(:,1),bocca(:,2),bocca(:,3), 'g', 'LineWidth', 2)
% hold on
% orecchio_sx = [p16;p17];
% lobo = p18;
% plot3(orecchio_sx(:,1),orecchio_sx(:,2),orecchio_sx(:,3), 'b', 'LineWidth', 2)
% hold on
% plot3(lobo(:,1),lobo(:,2),lobo(:,3), 'b', 'LineWidth', 2)
% hold on
% orecchio_dx = [p19; p20; p21];
% plot3(orecchio_dx(:,1),orecchio_dx(:,2),orecchio_dx(:,3), 'b', 'LineWidth', 2)

grid on
axis tight
axis equal


%% Dati del problema

d0 = 1;
a1 = 0.5;
a2 = 0.5;
d4 = 0.2;

L1 = Link('d',0,'a',a1,'alpha',0);
%L1 = Revolute('a', a1);
L2 = Link('d',0,'a',a2,'alpha',pi);
%L2 = Revolute('a', a2, 'alpha', pi);
L3 = Link('theta',0,'a',0,'alpha',0);
%L3 = Prismatic();
L3.qlim = [0 1];
L4 = Link('d',d4,'a',0,'alpha',0);
%L4 = Revolute('d', d4);

scara = SerialLink([L1 L2 L3 L4], 'name', 'SCARA'); 
scara.base = SE3(0, 0, d0);

%posizione iniziale dei giunti
T0=SE3(punto)*SE3.rpy(0, 0, -pi/4);
q0=scara.ikine(T0, 'mask', [1 1 1 1 0 0]);

 
%% Orientamento
phi_i = q0(1)+q0(2)-q0(4);
phi_f = 0;

[s, sd, sdd] = lspb(0,norm(phi_f-phi_i),t);

phi_e = phi_i + s*((phi_f-phi_i)/norm(phi_f-phi_i));
phi_e_d = sd*(phi_f-phi_i)/norm(phi_f-phi_i);
phi_e_dd = sdd*(phi_f-phi_i)/norm(phi_f-phi_i);

phi_e = phi_e';
phi_e_d = phi_e_d';
phi_e_dd = phi_e_dd';

traj = traj';
trajd = trajd';
trajdd = trajdd';

x = [traj; phi_e];
xd = [trajd; phi_e_d];
xdd = [trajdd; phi_e_dd];

figure(2)
plot(t,x,'LineWidth', 2)
grid on
axis tight
xlabel('tempo')
legend('coordinata x', 'coordinata y', 'coordinata z', 'orientamento')

%% simulazione con inversa del primo ordine
% K = diag([1200 1000 10 100]);
% r = sim('CLIK_inv_jacob_1order');
% figure(3)
% subplot(4,1,1);
% plot(r.t,r.q(:,1),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q1')
% 
% subplot(4,1,2);
% plot(r.t,r.q(:,2),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q2')
% 
% subplot(4,1,3);
% plot(r.t,r.q(:,3),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q3')
% 
% subplot(4,1,4);
% plot(r.t,r.q(:,4),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q4')
% 
% %errore
%figure(4)
% subplot(2,1,1);
% plot(r.t,r.err(:,1),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('norma errore')
% 
% subplot(2,1,2);
% plot(r.t,r.err(:,2),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('err oriantamento')
%% simulazione con trasposta
% K = diag([500 500 500 500]);
% r = sim('CLIK_trasp_jacob_1order');
% figure(3)
% subplot(4,1,1);
% plot(r.t,r.q(:,1),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q1')
% 
% subplot(4,1,2);
% plot(r.t,r.q(:,2),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q2')
% 
% subplot(4,1,3);
% plot(r.t,r.q(:,3),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q3')
% 
% subplot(4,1,4);
% plot(r.t,r.q(:,4),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q4')
% 
% %errore
% figure(4)
% subplot(2,1,1);
% plot(r.t,r.err(:,1),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('norma errore')
% 
% subplot(2,1,2);
% plot(r.t,r.err(:,2),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('err oriantamento')

%% simulazione con inversa del secondo ordine
kpi = 1500;
kdi = 300;
A = 1+Tc*kdi;
B = kpi*Tc*Tc-kdi*Tc-2;
delta = B^2-4*A;
s1 = (-B+sqrt(delta))/(2*A);
s2 = (-B-sqrt(delta))/(2*A);
KD = kdi*eye(4);
KP = kpi*eye(4);
% 
% r = sim('CLIK_inv_jacob_2order');
% figure(3)
% subplot(4,1,1);
% plot(r.t,r.q(:,1),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q1')
% 
% subplot(4,1,2);
% plot(r.t,r.q(:,2),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q2')
% 
% subplot(4,1,3);
% plot(r.t,r.q(:,3),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q3')
% 
% subplot(4,1,4);
% plot(r.t,r.q(:,4),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q4')
% 
% %errore
% figure(4)
% plot(r.t,r.err,'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('norma errore')
% legend('norma err posizione', 'err orientamento')

% %errore
% figure(4)
% subplot(2,1,1);
% plot(r.t,r.err(:,1),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('norma errore')
% 
% subplot(2,1,2);
% plot(r.t,r.err(:,2),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('err oriantamento')

%% CONTROLLO ROBUSTO
% wn =[20 23 10 50];
% zita =1;
% Kp = diag([wn(1)^2 wn(2)^2 wn(3)^2 wn(4)^2]);
% Kd = diag([2*zita*wn(1) 2*zita*wn(2) 2*zita*wn(3) 2*zita*wn(4)]);
% 
% D = [zeros(4,4); eye(4)];
% P = diag([1 1 1 1 1 1 1 1]);
% H = [zeros(4,4), eye(4); zeros(4,8)];
% K = [Kp Kd];
% Htilde = H-D*K;
% Q = lyap(Htilde,P);
% r = sim('ControlloRobusto');
% 
% figure(3)
% subplot(4,1,1);
% plot(r.t,r.q(:,1),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q1')
% 
% subplot(4,1,2);
% plot(r.t,r.q(:,2),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q2')
% 
% subplot(4,1,3);
% plot(r.t,r.q(:,3),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q3')
% 
% subplot(4,1,4);
% plot(r.t,r.q(:,4),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('q4')
% 
% %errore
% r.err(50002,:)= 1.0e-05 * [0.0002   -0.0001   -0.2490    0.0000];
% r.err(50003,:)= 1.0e-05 * [0.0002   -0.0001   -0.2490    0.0000];
% 
% figure(4)
% subplot(4,1,1);
% plot(r.t,r.err(:,1),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('errore q1')
% 
% subplot(4,1,2);
% plot(r.t,r.err(:,2),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('errore q2')
% 
% subplot(4,1,3);
% plot(r.t,r.err(:,3),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('errore q3')
% 
% subplot(4,1,4);
% plot(r.t,r.err(:,4),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('errore q4')
% 
% %coppie
% 
% figure(5)
% subplot(4,1,1);
% plot(r.t,r.tau(:,1),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('tau1')
% 
% subplot(4,1,2);
% plot(r.t,r.tau(:,2),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('tau2')
% 
% subplot(4,1,3);
% plot(r.t,r.tau(:,3),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('tau3')
% 
% subplot(4,1,4);
% plot(r.t,r.tau(:,4),'LineWidth', 2)
% grid on
% xlabel('tempo')
% title('tau4')

%% CONTROLLO ADATTATIVO
LAMBDA = 500*diag([1 1 1 1]);
Kd = 200*diag([1 1 1 1]);
r = sim('ControlloAdattativo');

figure(3)
subplot(4,1,1);
plot(r.t,r.q(:,1),'LineWidth', 2)
grid on
xlabel('tempo')
title('q1')

subplot(4,1,2);
plot(r.t,r.q(:,2),'LineWidth', 2)
grid on
xlabel('tempo')
title('q2')

subplot(4,1,3);
plot(r.t,r.q(:,3),'LineWidth', 2)
grid on
xlabel('tempo')
title('q3')

subplot(4,1,4);
plot(r.t,r.q(:,4),'LineWidth', 2)
grid on
xlabel('tempo')
title('q4')

%errore
figure(4)
subplot(4,1,1);
plot(r.t,r.err(:,1),'LineWidth', 2)
grid on
xlabel('tempo')
title('errore q1')

subplot(4,1,2);
plot(r.t,r.err(:,2),'LineWidth', 2)
grid on
xlabel('tempo')
title('errore q2')

subplot(4,1,3);
plot(r.t,r.err(:,3),'LineWidth', 2)
grid on
xlabel('tempo')
title('errore q3')

subplot(4,1,4);
plot(r.t,r.err(:,4),'LineWidth', 2)
grid on
xlabel('tempo')
title('errore q4')

%coppie

figure(5)
subplot(4,1,1);
plot(r.t,r.tau(:,1),'LineWidth', 2)
grid on
xlabel('tempo')
title('tau1')

subplot(4,1,2);
plot(r.t,r.tau(:,2),'LineWidth', 2)
grid on
xlabel('tempo')
title('tau2')

subplot(4,1,3);
plot(r.t,r.tau(:,3),'LineWidth', 2)
grid on
xlabel('tempo')
title('tau3')

subplot(4,1,4);
plot(r.t,r.tau(:,4),'LineWidth', 2)
grid on
xlabel('tempo')
title('tau4')

figure(6)
plot(r.t,r.param,'LineWidth', 2)
grid on
xlabel('tempo')
title('carico')

return;
figure(5)
qp=r.q;
qprid=qp(1:100:end,:);
L={'r', 'LineWidth', 2};
scara.plot(qprid,'trail',L)