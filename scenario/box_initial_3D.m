function [nBoxObs, box_pos, box_size, box_yaw] = box_initial_3D(mode)

    switch mode
        case 0 
            nBoxObs = 0;
            box_pos = [];
            box_size = [];
            box_yaw = [];
        case 1
            nBoxObs   = 1;

            box_pos   = zeros(3, nBoxObs);
            box_size  = zeros(3, nBoxObs);
            box_yaw   = zeros(1, nBoxObs);

            box_pos(:, 1)  = [-4; 1; 2];
            box_size(:, 1) = [3; 2; 4];
            box_yaw(:, 1)  = deg2rad(0);
        case 3
            nBoxObs   = 3;

            box_pos   = zeros(3, nBoxObs);
            box_size  = zeros(3, nBoxObs);
            box_yaw   = zeros(1, nBoxObs);

            box_pos(:, 1)  = [-4; 1; 2];
            box_size(:, 1) = [3; 2; 4];
            box_yaw(:, 1)  = deg2rad(0);

            box_pos(:, 2)  = [3.0; 4.0; 4.0];
            box_size(:, 2) = [2.0; 3.6; 8.0];
            box_yaw(:, 2)  = deg2rad(-45);

            box_pos(:, 3)  = [0.0; -4; 5.0];
            box_size(:, 3) = [1.6; 3.2; 3.0];
            box_yaw(:, 3)  = deg2rad(0);
        otherwise
            error('Obstacle positions initialization failed!');
    end
end
