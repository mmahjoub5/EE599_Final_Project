function u_nom = get_nominal_controller(xinit,params)
    % return the mpc based nominal controller
    X = mpc(params.H, params.goalX, params.goalY, params.dt, params.wMax, xinit);
    u_nom = [X(4*(params.H+1)+2); X(5*(params.H+1)+1)];
end