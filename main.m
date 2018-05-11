function main

clc
clf
figure(gcf) % bring figure to front

% Switch the parameter set to select the SL or SCR method.
p = params('SL','Track1');
%p = params('SCR','Track1');

x0 = [0 0 0 0]'; % Initial state vector [px py vx vy]
controller_output = [];

while true

    % Controller
    controller_output = p.controller(p,x0,controller_output);    
    
    % Visualization
    plotOptimizationResult(p, [x0 controller_output.x], controller_output);
    drawnow
    
    % "Simulation"
    % Assumes that the vehicle follows the predicted trajectory exactly.
    % A more accurate and complex vehicle dynamics model could be used here.
    ddt = p.dt * p.dt / 2;
    Ad = [1 0 p.dt 0; 0 1 0 p.dt; 0 0 1 0; 0 0 0 1];
    Bd = [ddt 0; 0 ddt; p.dt 0; 0 p.dt];
    u = controller_output.U(:,1);
    
    x0 = Ad * x0 + Bd * u;
end

end