close; clc; clear;
%% Definition of Robot Structure

% Defining the DH parameters for the robot
DH = [0     76     0     pi/2;
      0     -23.65 43.23 0;
      0     0      0     pi/2;
      0     43.18  0     -pi/2;
      0     0      0     pi/2;
      0     20     0     0];

% Creating robot structure
myrobot = mypuma560(DH);

%% Plot a simple joint space trajectory
% Creating empty q matrix
q = zeros(200, 6);

% Creating steps for each column
q(:,1) = linspace(0, pi, 200);
q(:,2) = linspace(0, pi/2, 200);
q(:,3) = linspace(0, pi, 200);
q(:,4) = linspace(pi/4, 3*pi/4, 200); 
q(:,5) = linspace(-pi/3, pi/3, 200);
q(:,6) = linspace(0, 2*pi, 200);

% Plotting robot evolution corresponding to joint angles in q
plot(myrobot, [0,0,0,0,0,0])

%% Forward Kinematics

% H = forward(q, myrobot)


