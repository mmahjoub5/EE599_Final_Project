function u_filtered = get_safety_controller(x_curr,u_nom,params)  
   
    %% TODO
    % Code in the least restrictive safety filter that returns the safe
    % controller if the nominal controller leads the next state into an
    % unsafe region
    
%     obsX = params.obsX1;
%     obsY = params.obsY1;
   
    next_state = simulate(x_curr, u_nom, 0, params.dt, params.noise_drift);
    
     
    val = eval_u(params.g,params.data(:,:,:,:,end),next_state);
    if val < 0 
        disp("printing val")
        disp(val);
    end 
    
    if val <= 0.1 % safety condition)  % the system will end in unsafe state!
            u_filtered = eval_u(params.g,params.safety_controller,x_curr); % get safety controller
    else
            u_filtered = u_nom;
    end

end