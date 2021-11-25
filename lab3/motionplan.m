function qref = motionplan(q0, q_f, t1, t2, myrobot, obs, tol)
       
    % q0, q1, .., qi, ..., qN, with i = 1, ..., N and qN = q_f
    % q : N x 6 matrix, where each row is qi^T

    q = q0;

    while norm(q(end,1:5)'-q_f(1:5)') >= tol
        
          % current vector qi
          qi = q(end, 1:6)';
         
          % compute new qii = q(i+1)
          tau = att(qi, q_f, myrobot); % vector 6x1
          
           fprintf("Close to objects:");
           cobjs = [];
          for c = 1:size(obs,2)
              rep_c = rep(qi, myrobot, obs{c});
              tau = tau + 100 * rep_c; % Objects are dangerous
              if norm(rep_c) ~= 0
                  cobjs(end+1) = c;
              end
          end
          disp(cobjs);
   
          alpha = 0.01;
          qii = qi + alpha * tau;     

          % save new qii to the matrix q
          q(end+1, 1:6) = qii';
    
          % display
          fprintf("Step " + (size(q,1)-1)  + ", error of " + norm(q(end,1:5)'-q_f(1:5)')+ "\n");
          disp(qii')

          fprintf("\n \n");
    end
    
    % fill the last column for the 6th joint variable with a linespace
    q(:,6) = linspace(q0(6), q_f(6), size(q,1));

    % spline
    t = linspace(t1, t2, size(q,1));
    qref = spline(t, q');    
end

