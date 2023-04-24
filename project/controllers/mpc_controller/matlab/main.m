clear all; close all; clc;
%% Get the params
params = get_params();

%%

load("saved_data/params.mat");
%%
xinit = [-2.5;-1;0;1];
traj = [xinit];
% y = RMPC(10, 100000,xinit);

while true
    y = RMPC(50, 2000, xinit)
    if (xinit(1) - params.goalX)^2 < 0.01 && (xinit(2) - params.goalY)^2 < 0.01
        break
    end
   
    if y ~= [-999999]
        result = fourDim_dynamics(xinit(1), xinit(2), xinit(3), xinit(4),y(1),y(2), 0.01);
        traj = [traj, result'];
        xinit = result'
        plotTraj(traj)
    else
        break
    end
end



%%
xinit = [0;2;0;1];
traj = [xinit];
while true
    %y = RMPC(50, 2000, xinit);
    if (xinit(1) - params.goalX)^2 < 0.1 && (xinit(2) - params.goalY)^2 < 0.1
        break
    end
    X = mpc(params.H, params.goalX, params.goalY, params.dt, params.wMax, xinit);
    w = X(4*(params.H+1)+2);
    a = X(5*(params.H+1)+1);
    %result = fourDim_dynamics(xinit(1), xinit(2), xinit(3), xinit(4),w,a, params.dt);
    result = fourDim_dynamics(xinit(1), xinit(2), xinit(3), xinit(4),y(1),y(2), 0.01, [0, 0]);
    traj = [traj, result];
    xinit = result
end 


plot(traj(1, :), traj(2, :), 'color', 'b', 'LineWidth', 2);
%%
close all;


plot(traj(1, :), traj(2, :), 'color', 'b', 'LineWidth', 2);
hold on;
xline(1)
xline(5)
yline(1)
yline(5)
xline(-1)
xline(-5)
yline(-1)
yline(-5)
hold off
xlim([-10, 10]);
ylim([-10, 10])
%%

u_nom = get_nominal_controller(current_state,params);

%% TODO: Reachability analysis: Please complete the template in the BRT_computation.m file for this section to get the params
% disp('Pre-computing the safety controller with the BRT.........................')
[params.safety_controller, params.worst_dist, params.data, params.tau, params.g, params.derivatives] = BRT_computation(params); % the BRT gives us the safety controller for free


%% Simulate trajectory with online filtering
% The robot trajectory initialized
traj = [params.xinit];
cont_traj = [;];


while ~stopping_criteria(traj(:,end),params)
    
    current_state = traj(:,end);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % test noise generation
    params.noise_drift = tests(current_state,params);
    disturbance = norm(params.noise_drift);  % magitude of disturbance
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    params.d = disturbance;
    
    % get the nominal controller: This is given to you 
    u_nom = get_nominal_controller(current_state,params);
    switch params.controller_choice
        case 0
            cont_traj = [cont_traj; u_nom]; 
            chosen_controller = u_nom;
        case 1
            % TODO get the least restrictive safety filtered controller
            chosen_controller = get_safety_controller(current_state,u_nom,params);
        case 2
            % TODO get the quadratic program filtered controller
            chosen_controller = get_qpfilter_controller(current_state,u_nom,params);
    end
    
    % update the control trajectory
    if params.controller_choice > 0
        cont_traj = [cont_traj;u_nom chosen_controller];
    end
    
    % noisy update to the next state using the chosen control
    next_state = simulate(current_state, chosen_controller, 0, params.dt, params.noise_drift);
    
    traj = [traj, next_state']; % update the trajectory
    plot_env(traj,params); % plot the env after every step
    hold on 
    yline(-40, 'Color', 'r');
    yline(-20, 'Color', 'r');
    pause(0.05)
end

% plotting the controllers
plot_controller(params.controller_choice, cont_traj);

%%

function y = plotTraj(traj)
    figure(1);
    plot(traj(1, :), traj(2, :), 'color', 'b', 'LineWidth', 2);
    hold on;
    xline(1)
    xline(5)
    yline(1)
    yline(5)
    xline(-1)
    xline(-5)
    yline(-1)
    yline(-5)
    hold off
    xlim([-10, 10]);
    ylim([-10, 10])

end