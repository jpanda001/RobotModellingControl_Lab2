function tau = att(q, q_f, myrobot)

    % returns a tau vector (6x1)

    tau = zeros(6,1);
    Fatts = zeros(3,6);
    
    %% loop through all the links
    for i = 1:6
    
        % find velocity jacobian
        Jv_i = zeros(3,6);
    
        % end effector
        Hi = myrobot.A(1:i, q);
        oi = Hi.t;
    
        % fill the colums of the velocity jacobian until column i
        for j = 1:i
            Hj_1 = myrobot.A(1:j-1,q);
            Rj_1 = Hj_1.R;
            zj_1 = Rj_1(:,3);
            oj_1 = Hj_1.t;
            Jv_i(:,j) = cross(zj_1, (oi - oj_1));
        end
    
        oi_f = myrobot.A(1:i, q_f).t;
    
        Fatti = -(oi - oi_f);
        tau = tau + Jv_i' * Fatti;
    
        Fatts(:,i) = Fatti;
    end
    
    if tau ~= 0
        tau = tau/norm(tau);
    end
end