function collision_mtx = collision_check(nRobot, nBoxObs, robot_pos, robot_radius, ...
    box_vert)
    % 
    % Check if collision happends among multiple agents with obstacles
    % 
    
    dim = size(robot_pos, 1);
    
    %% inter robot 
    collision_mtx_robot = zeros(nRobot, nRobot);
    for i = 1 : nRobot-1
        for j = (i+1) : nRobot
            pos_ij = robot_pos(:, i) - robot_pos(:, j);
            size_ij = 2 * robot_radius;
            d_ij = norm(pos_ij);
            if d_ij < size_ij*1
                collision_mtx_robot(i, j) = 1;
            end
            collision_mtx_robot(j, i) = collision_mtx_robot(i, j);
        end
    end
    
    %% robot obstacle    
    collision_mtx_obs = zeros(nRobot, nBoxObs);
    for i = 1 : nRobot
        for j = 1 : nBoxObs
            pos_i = robot_pos(:, i);        % d*1
            vert_j = box_vert(:, :, j);     % d*n
            if dim == 2
                dis_ij = p_poly_dist(pos_i(1), pos_i(2), vert_j(1, :), vert_j(2, :));
            elseif dim == 3
                dis_ij = openGJK(pos_i, vert_j);
            else
                error('Dimension error!');
            end
            if dis_ij < robot_radius*1
                collision_mtx_obs(i, j) = 1;
            end
        end
    end
    
    collision_mtx = [collision_mtx_robot, collision_mtx_obs];
    
end


