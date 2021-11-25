function tau = rep(q, myrobot, ob)
    
    % returns a tau vector (6x1)

    tau = zeros(6,1);   
    Freps = zeros(3,6);

    %% loop through all the links
    for i = 1:6
       
        % find velocity jacobian
        Jv_i = zeros(3,6);
    
        % end effector
        Hi = myrobot.A(1:i, q);
        oi = Hi.t;
    
        % fill the colums of the velocity jacobian until column i
        for j = 1:i 
            Hj_1 = myrobot.A(1:j-1, q);
            Rj_1 = Hj_1.R;
            zj_1 = Rj_1(:,3);
            oj_1 = Hj_1.t;
            Jv_i(:,j) = cross(zj_1, (oi - oj_1));   
        end
           
        Frepi = zeros(3,1);
        if strcmp(ob.type, 'cyl')
            oixy = oi(1:2);
            dist = norm(oixy - ob.c) - ob.R;
            % already in collision
            if dist < ob.R
                fprintf("Already in collision \n");
            % outside of the region of influence
            elseif dist > ob.rho0
                Frepi = zeros(3,1);
            % in the region of influence
            else           
                r2d = oixy - ob.c;  
                r = [r2d(1) r2d(2) 0]';
                r = r / norm(r);
                Frepi = 1 * (1/dist - 1/ob.rho0) / dist^2 * r;
            end

        elseif strcmp(ob.type, 'sph')
            dist = norm(oi - ob.c) - ob.R;
            % already in collision
            if dist < ob.R
                fprintf("Already in collision \n");
            % outside of the region of influence
            elseif dist > ob.rho0
                Frepi = zeros(3,1);
            % in the region of influence
            else           
                r = oi - ob.c;
                r = r / norm(r);
                Frepi = 1 * (1/dist - 1/ob.rho0) / dist^2 * r;
            end
        else
            fprintf("Strange object \n");
        end
       
        tau = tau + Jv_i' * Frepi;    

        Freps(:,i) = Frepi;
    end

    tau = tau * 100;

%     if tau ~= 0
%         tau = tau/norm(tau);
%     end

end