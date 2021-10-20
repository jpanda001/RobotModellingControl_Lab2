function H = forward(joint, myrobot)
   
    % get DH table
    n = length(myrobot.a);
    DH = zeros(n,4);
    
    DH(:,2) = myrobot.d';
    DH(:,3) = myrobot.a';
    DH(:,4) = myrobot.alpha'; 
    
    function R = myrotz(theta)
        R = [
            cos(theta) -sin(theta) 0;
            sin(theta) cos(theta)  0;
            0          0           1;
        ];
    end
    
    function R = myrotx(theta)
        R = myrotz(theta);       
        Y = [
            0 0 1;
            0 1 0;
            -1 0 0;
        ];
        R = Y * R * Y';
        
    end
    
    function H = rotToTrot(R)
       H = eye(4,4);
       H(1:3,1:3) = R;
    end

    function H = mytrotz(theta) 
        H = rotToTrot(myrotz(theta));
    end

    function H = mytrotx(theta) 
        H = rotToTrot(myrotx(theta));
    end

    
    function H = mytransl(x,y,z)
        H = eye(4,4);
        H(1:3,4) = [x y z]';
    end


  
   % DH 6x4 matrix where theta, di, ai, alphai
   H = eye(4,4);
    for i = 1:length(joint)
        H = H * mytrotz(joint(i)) * mytransl(0,0,DH(i,2))* mytransl(DH(i,3),0,0)*mytrotx(DH(i,4));       
    end
        

end

