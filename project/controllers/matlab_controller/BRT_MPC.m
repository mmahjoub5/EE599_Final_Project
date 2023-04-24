function y = BRT_MPC(current_state, params, cont_traj)
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
%             cont_traj = [cont_traj; u_nom]; 
            chosen_controller = u_nom;
        case 1
            % TODO get the least restrictive safety filtered controller
            chosen_controller = get_safety_controller(current_state,u_nom,params);
        case 2
            % TODO get the quadratic program filtered controller
            chosen_controller = get_qpfilter_controller(current_state,u_nom,params);
    end
    
    % update the control trajectory
%     if params.controller_choice > 0
%         cont_traj = [cont_traj;u_nom chosen_controller];
%     end

    y = [chosen_controller cont_traj];

end