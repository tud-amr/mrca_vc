function [nBoxObs, box_pos, box_size, box_yaw] = box_initial_2D(mode)

    switch mode
        case 0 
            nBoxObs = 0;
            box_pos = [];
            box_size = [];
            box_yaw = [];
        case 1
            nBoxObs   = 1;
            
            box_pos   = zeros(2, nBoxObs);
            box_size  = zeros(2, nBoxObs);
            box_yaw   = zeros(1, nBoxObs);

            box_pos(:, 1)  = [-4; 1];
            box_size(:, 1) = [3; 2];
            box_yaw(:, 1)  = deg2rad(0);
        case 3
            nBoxObs   = 3;

            box_pos   = zeros(2, nBoxObs);
            box_size  = zeros(2, nBoxObs);
            box_yaw   = zeros(1, nBoxObs);

            box_pos(:, 1)  = [-4; 1];
            box_size(:, 1) = [1; 1];
            box_yaw(:, 1)  = deg2rad(0);

            box_pos(:, 2)  = [3.0; 4.0];
            box_size(:, 2) = [1; 1];
            box_yaw(:, 2)  = deg2rad(0);

            box_pos(:, 3)  = [0.0; -2];
            box_size(:, 3) = [1; 1];
            box_yaw(:, 3)  = deg2rad(0);
        case 10
            nBoxObs     = 10;
            
            box_pos   = zeros(2, nBoxObs);
            box_size  = ones(2, nBoxObs);
            box_yaw   = zeros(1, nBoxObs);

            box_pos(:, 1)  = [-3; 4.5];
            box_size(:, 1) = [1; 1];
            box_yaw(:, 1)  = deg2rad(0);

            box_pos(:, 2)  = [-2; 3.5];
            box_size(:, 2) = [1; 1];
            box_yaw(:, 2)  = deg2rad(0);

            box_pos(:, 3)  = [-4.5; 1];
            box_size(:, 3) = [1; 1];
            box_yaw(:, 3)  = deg2rad(0);
            
            box_pos(:, 4)  = [-1; -4.5];
            box_size(:, 4) = [1; 1];
            box_yaw(:, 4)  = deg2rad(0);
            
            box_pos(:, 5)  = [-2.5; -1.5];
            box_size(:, 5) = [1; 1];
            box_yaw(:, 5)  = deg2rad(0);
            
            box_pos(:, 6)  = [1.5; -2.5];
            box_size(:, 6) = [1; 1];
            box_yaw(:, 6)  = deg2rad(0);
            
            box_pos(:, 7)  = [4.5; -0.5];
            box_size(:, 7) = [1; 1];
            box_yaw(:, 7)  = deg2rad(0);
            
            box_pos(:, 8)  = [2.5; 3.5];
            box_size(:, 8) = [1; 1];
            box_yaw(:, 8)  = deg2rad(0);
            
            box_pos(:, 9)  = [0.5; 0.5];
            box_size(:, 9) = [1; 1];
            box_yaw(:, 9)  = deg2rad(0);
            
            box_pos(:, 10)  = [-4.5; -4.5];
            box_size(:, 10) = [1; 1];
            box_yaw(:, 10)  = deg2rad(0);
            
        otherwise
            error('Obstacle positions initialization failed!');
    end
end
