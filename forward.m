%% Function that creates the forward Homogenous Transformation matrix
function H = forward(joint, myrobot)
    
    H = myrobot.A(1:6, joint)

end