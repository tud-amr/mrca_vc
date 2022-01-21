function robotStartPos = robotStartPos_3D(nRobot, mode, xDim, yDim, zDim, radius)

    robotStartPos = zeros(3, nRobot);
    
    diameter = 2*radius;    % robot diameter
    
    switch mode
        case 1                          % uniformly distributed in a circle
            angle = 2.0*pi / nRobot; 
            R = 0.4*(xDim(2)-xDim(1));
            for iRobot = 1 : nRobot
                angle_i = deg2rad(0) + angle*(iRobot-1);
                robotStartPos(1:2, iRobot) = [R*cos(angle_i); R*sin(angle_i)];
                robotStartPos(3, iRobot) = zDim(1) + 0.5*(zDim(2)-zDim(1));
            end
        case 2                          % complete random position
            % discrete the workspace
            xL = xDim(2) - xDim(1);
            yL = yDim(2) - yDim(1);
            zL = zDim(2) - zDim(1);
            xN = floor(xL / diameter) - 2;
            yN = floor(yL / diameter) - 2;
            zN = floor(zL / diameter) - 2;
            % generate random integer number
            for iRobot = 1 : nRobot
                xN_i = randi([1, xN-1], 1);
                yN_i = randi([1, yN-1], 1);
                zN_i = randi([1, zN-1], 1);
                robotStartPos(1, iRobot) = xDim(1) + diameter*xN_i;
                robotStartPos(2, iRobot) = yDim(1) + diameter*yN_i;
                robotStartPos(3, iRobot) = zDim(1) + diameter*zN_i;
            end
        case 3
            angle = 2.0*pi / nRobot;
            ang_gap = 0.4*angle;
            R_min = xDim(2) - 3;
            R_max = xDim(2) - 0.5;
            for iRobot = 1 : nRobot
                % initial
                angle_c = deg2rad(0) + angle*(iRobot-1);
                angle_min = angle_c - ang_gap;
                angle_max = angle_c + ang_gap;
                angle_i = angle_min + (angle_max - angle_min) * rand;
                R_i = R_min + (R_max - R_min) * rand;        
                robotStartPos(1:2, iRobot) = [R_i*cos(angle_i); R_i*sin(angle_i)];
                robotStartPos(3, iRobot) = zDim(1) + 0.5*(zDim(2)-zDim(1));
            end
        otherwise
            error('Robot starting positions initialization failed!');
    end




end