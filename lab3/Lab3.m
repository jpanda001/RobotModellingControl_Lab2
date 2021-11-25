clc
clear
close all

%% Configuring robot

DH = [
      0     76     0     pi/2; 
      0     -23.65 43.23 0;
      0     0      0     pi/2;
      0     43.18  0     -pi/2;
      0     0      0     pi/2;
      0     20     0     0
];

myrobot = mypuma560(DH);


%% Define start and end points

H1 = eul2tr([0 pi pi/2]);
H1(1:3, 4) = 100 * [-1 ; 3 ; 3] / 4;
q1 = inverse(H1, myrobot)

H2 = eul2tr([0 pi -pi/2]);
H2(1:3, 4) = 100 * [3; -1; 2] / 4;
q2 = inverse(H2, myrobot)


%% Tests

setupobstacle();

% Attractive tau test
tau = att(q1, q2, myrobot)';
disp(tau);

% Repulsive tau test
q3 = 0.9*q1 + 0.1*q2;
tau = rep(q3, myrobot, obs{1})';
disp(tau/norm(tau));

q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q, myrobot, obs{6})';
disp(tau/norm(tau));


%% Motion planning

setupobstacle();
hold on;
axis([-100 100 -100 100 0 200]);
view(-32, 50);
plotobstacle(obs);

qref = motionplan(q1, q2, 0, 10, myrobot, obs, 0.01);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot, q);
hold off;
