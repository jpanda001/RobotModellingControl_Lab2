% init
clc, clear


%% 4.1 DH Table and robot initialization
fprintf("-----------------------------------------\n");
fprintf("4.1 Defining the robot :\n");
DH = [
    0     76     0     pi/2;
      0     -23.65 43.23 0;
      0     0      0     pi/2;
      0     43.18  0     -pi/2;
      0     0      0     pi/2;
      0     20     0     0];

fprintf("DH: \n");
disp(DH);

% get robot
myrobot = mypuma560(DH)

%% 4.2 Sample trajectory
fprintf("-----------------------------------------\n");
fprintf("4.2 Sample trajectory : \n");

% qs 200x6 matrix
n = 200;
qs = zeros(n,6);
qs(:,1) = linspace(0,pi,n);
qs(:,2) = linspace(0,pi/2,n);
qs(:,3) = linspace(0,pi,n);
qs(:,4) = linspace(pi/4,3/4*pi,n);
qs(:,5) = linspace(-pi/3,pi/3,n);
qs(:,6) = linspace(0,2*pi,n);


% plot !! Activate !!
%plot(myrobot, qs)


%% 4.3 Forward kinematic
fprintf("-----------------------------------------\n");
fprintf("4.3 Forward kinematic :\n");

% get the end effector position and the matrices H
Hs = zeros(4, 4, n);
os = zeros(200,3);
for t = 1:n
    H = forward(qs(t,:), myrobot); 
    Hs(:, :, t) = H;
    os(t,:) = H(1:3,4);
end


% plot !! Activate !!
% plot3(os(:,1), os(:,2), os(:,3),'r');
% hold on
% plot(myrobot, qs)

%% 4.4 Inverse kinematic
fprintf("-----------------------------------------\n");
fprintf("4.4 Inverse kinematic :\n");

H = [
    cos(pi/4) -sin(pi/4) 0 20 ;
    sin(pi/4) cos(pi/4) 0 23;
    0         0         1  15;
    0         0         0   1;
 ];

q_inv =  inverse(H, myrobot);
fprintf("q from the inverse kinematic that should satisfy H : \n");
fprintf("q: \n");
disp(q_inv')
fprintf("H: \n");
disp(H);


%% Table exercise

% define d matrix with the position of the end effector
n = 100;
d = zeros(n,3);
d(:,1) = linspace(10,30,n);
d(:,2) = linspace(23,30,n);
d(:,3) = linspace(15,100,n);

% end effector orientation
R = rotz(pi/4);

qs_inv = zeros(n,6);
os = zeros(n,3);
for i = 1:n  
   % set H
   H = eye(4,4);
   H(1:3,1:3) = R;
   H(1:3,4) = d(i,1:3)';
   
   % get the qs that satisfy H
   qs_inv(i,:) = inverse(H, myrobot);    
   
   %  get the end effector position
   H = forward(qs_inv(i,:), myrobot); 
   os(i,:) = H(1:3,4);
end


% plot !! activate !!
plot3(os(:,1), os(:,2), os(:,3),'r');
hold on
plot(myrobot, qs_inv);

