function checkpoints = testTrack1

    trackWidth = 7;

    checkpoints = struct;
    checkpoints.left = [0; trackWidth/2];
    checkpoints.right = [0; -trackWidth/2];
    checkpoints.center = [0; 0];
    checkpoints.yaw = 0;
    checkpoints.forward_vector = [1; 0];

    checkpoints = add_turn(checkpoints, 0, 76, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 50, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 8, trackWidth);
    checkpoints = add_turn(checkpoints, -0.1, 30, trackWidth);
    checkpoints = add_turn(checkpoints, 0.1, 30, trackWidth);
    checkpoints = add_turn(checkpoints, 0.5, 15, trackWidth);
    checkpoints = add_turn(checkpoints, -0.5, 30, trackWidth);
    checkpoints = add_turn(checkpoints, 0.5, 15, trackWidth);
    checkpoints = add_turn(checkpoints, 0, 60, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 10, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 20, trackWidth);
    checkpoints = add_turn(checkpoints, 0, 55, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 90.3, trackWidth);
    checkpoints = add_turn(checkpoints, 0, 5.3, trackWidth);
    checkpoints = add_turn(checkpoints, -0.25, 20, trackWidth);
    
    checkpoints = checkpoints(2:end);
end



function checkpoints = add_turn(checkpoints, phi, L, width)

    kappa = (phi*(2*pi))/L;
    N = 40;
    ds = L / N;
    
    for i=1:N
        checkpoints(end+1).yaw = checkpoints(end).yaw + kappa * ds;
        c = cos(checkpoints(end).yaw);
        s = sin(checkpoints(end).yaw);
        f = [c;s];
        n = [-s;c];
        checkpoints(end).center = checkpoints(end-1).center + f * ds;
        checkpoints(end).left = checkpoints(end).center + n * width/2;
        checkpoints(end).right = checkpoints(end).center - n * width/2;
        checkpoints(end).forward_vector = f;
        
    end

end