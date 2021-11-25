function q = inverse(H, myrobot)
    
    % get DH table
    n = length(myrobot.a);
    DH = zeros(n,4);
    
    DH(:,2) = myrobot.d';
    DH(:,3) = myrobot.a';
    DH(:,4) = myrobot.alpha'; 
    
    % define parameters
    o_d0 = H(1:3,4);
    R_d = H(1:3,1:3);
    d1 = DH(1,2);
    d2 = DH(2,2);
    a2 = DH(2,3);
    d4 = DH(4,2);
    d6 = DH(6,2);
    
    % get oc vector
    o_c0 = o_d0 - R_d * [0 0 d6]';
    ocx = o_c0(1);
    ocy = o_c0(2);
    ocz = o_c0(3);
     
    % get theta1-3
    theta1 = atan2(ocy, ocx) - asin(-d2/sqrt(ocx^2 + ocy^2));  
    theta3 = -1/2*pi + acos((a2^2 + d4^2 - (ocx^2 + ocy^2  -d2^2 +(ocz -d1)^2) ) / (2 * a2 * d4) );  
    theta2 = atan2(ocz-d1, sqrt(ocx^2 + ocy^2 -d2^2)) - atan2(sin(theta3-pi/2)*d4,a2 + cos(theta3-pi/2)*d4);
    
    % H_03 from foward equations using theta1, theta2, theta3
    H_03 = myrobot.A(1:3, [theta1 theta2 theta3]);
    R_03 = H_03.R;
    
    % compute R_36
    R_36 = R_03' * R_d;
    
    % using the Euler angles for the spherical wrist problem    
    theta456 = tr2eul(R_36);
    
    % put all the theta together
    q = [theta1 theta2 theta3 theta456(1) theta456(2) theta456(3)];  
end

