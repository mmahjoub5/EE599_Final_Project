wb_console_print(sprintf('Hello World!\n'), WB_STDOUT);
car_node = wb_supervisor_node_get_from_def('car');
car_field = wb_supervisor_node_get_field(car_node,'translation');
p = load("saved_data/params_latest_4_23.mat"); % not the right file 
params = p.params;
cont_traj = [;];
% controller time step
TIME_STEP = 40;
car_pos = wb_supervisor_field_get_sf_vec3f(car_field);
xinit = [car_pos(1); car_pos(2); 0; 1];
traj = [xinit];
z = car_pos(3);
display(params)
while wb_robot_step(TIME_STEP) ~= -1
   chosen_controller = BRT_MPC(xinit, params, cont_traj);
   next_state = simulate(xinit, chosen_controller(1:2), 0, params.dt, [0 0]);
   display(next_state)
   xinit = next_state;
   traj = [traj xinit'];
   wb_supervisor_field_set_sf_vec3f(car_field, [next_state(1) next_state(2) z])
    % wb_console_print(car_pos, WB_STDOUT);
    
    display(class(car_pos))
    display(car_pos(1))
    %cont_traj = chosen_controller(3);
end