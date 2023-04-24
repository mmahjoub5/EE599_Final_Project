 wb_console_print(sprintf('Hello World!\n'), WB_STDOUT);
  car_node = wb_supervisor_node_get_from_def('car');
  car_field = wb_supervisor_node_get_field(car_node,'translation');
  % controller time step
  TIME_STEP = 40;

while wb_robot_step(TIME_STEP) ~= -1
    
   car_pos = wb_supervisor_field_get_sf_vec3f(car_field);
    % wb_console_print(car_pos, WB_STDOUT);
    
    
end