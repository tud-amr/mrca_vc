function cost = si_objective(z, p)
    
    % define stage cost function for mpc
    
    global index pr                         % global index information
    
    %% obtaining necessary information
    % cost terms weights
    w_wp_pos    =   p(index.p.weights(1));  % waypoint cost, pos
    w_input_v   =   p(index.p.weights(2));  % control inputs cost, v
    % ego robot 
    ego_pos     =   z(index.z.pos);         % current stage position [x, y]
%     ego_slack   =   z(index.z.slack);       % slack
    ego_input   =   z(index.z.inputs);      % control input [v_x, v_y]
    ego_start   =   p(index.p.startPos);    % start [x0, y0]
    ego_goal    =   p(index.p.wayPoint);    % goal [xg, yg]
    ego_v_max   =   p(index.p.speedLimit);  % maximal speed
    
    %% waypoint cost
    % goal position
    cost_wp_pos = w_wp_pos * obj_desired_pos(pr.dim, ego_pos, ego_start, ego_goal);

    %% control input cost
    cost_input = w_input_v * ego_input'*ego_input/(ego_v_max^2);
    
    %% slack cost
%     cost_slack = 10^4 * ego_slack * ego_slack;

    %% combine all cost
    cost = cost_wp_pos + cost_input;
%     cost = cost_wp_pos + cost_input + cost_slack;
    

end
