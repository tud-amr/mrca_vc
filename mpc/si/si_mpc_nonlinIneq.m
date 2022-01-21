function ineq = si_mpc_nonlinIneq(z, p, nHyperplanes)

    % define nonlinear inequalities for mpc

    global index pr                         % global index information
    
    %% obtaining necessary information
    % ego robot 
    ego_pos     =   z(index.z.pos);         % current stage position [x, y]
%     ego_slack   =   z(index.z.slack);       % slack variable
    ego_vel     =   z(index.z.inputs);      % [vx, vy]
    % hyperplanes
    safe_region =   p(index.p.safeRegion);  
    % speed limit
    ego_v_max   =   p(index.p.speedLimit);  % maximal speed
    
    %% speed limit
    cons_speed = ego_vel'*ego_vel - ego_v_max;
    
    %% motion constrained in safe region
    cons_coll_safe_region = [];
    for iHyperplane = 1 : nHyperplanes
        p_plane = safe_region(:, iHyperplane);
        a = p_plane(1:pr.dim);
        b = p_plane(pr.dim+1);
%         cons_plane = b - a'*ego_pos + ego_slack;
        cons_plane = b - a'*ego_pos;
        cons_coll_safe_region = [cons_coll_safe_region; cons_plane];
    end
    
    %% inequality constraints
    ineq = [cons_speed; cons_coll_safe_region];
end
