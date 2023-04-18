function params = get_params()
    %% Pack all params

    % Dynamics parameters
    params.nX = 3; % States - [p_x, p_y, theta]
    params.nU = 1; % Controls - [w]
    params.dt = 0.05; % Discretization step
    params.speed = 4; % Maximum speed - 1 m/s
    params.wMax = pi; % Maximum control - 1.1 rad/s
    params.xinit = [-40; -40; 0; 0]; % Initial state

    % Setup environment parameters
    % goal params  
    params.goalX = 40;
    params.goalY = 40;
    params.goalR = 2;

    % obstacle 1 params
    params.obsX1 = -5;
    params.obsY1 = 0;
    params.obswidth1 = 0.0;
    params.obsheight1 = 0.0;

    % obstacle 2 params
    params.obsX2 = 1;
    params.obsY2 = 0;
    params.obswidth2 = 0.0;
    params.obsheight2 = 0.0;

    % disturbance params 
    params.dMax = 0.8;

    % bonus: disturbance range
    params.dist_min = 0.1;
    params.dist_max = 0.8;

    % mpc params
    params.H = 10; % receeding control horizon
    
    % set controller choice: to be set by user
    params.controller_choice = 1; % choice 0 -> nominal, 1 -> least restrictive, 2 -> qp
    
    % test case: to be set by user
    params.test_choice = 1;
    
    
    % preload the optDist file for test case 4,5
    switch params.test_choice
        case 4
             [params.precom_optDist_g, params.precom_optDist_values] = ...
                 get_optDst_precom("optimal_disturbance/optDst_0_1.mat");
        case 5
             [params.precom_optDist_g, params.precom_optDist_values] = ...
                 get_optDst_precom("optimal_disturbance/optDst_0_8.mat");
    end    
end