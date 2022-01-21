%% initialization script
% common pre-run set up for simulation and experiment

%% Setup path
setPath;


%% Problem setup
global pr
% environment dimension
pr.dim = mode_dim;                % problem dimension
xdim = [-5, 5];                   % workspace
ydim = [-5, 5];
zdim = [ 0, 3];
if pr.dim == 2
    pr.ws = [xdim; ydim];
elseif pr.dim == 3
    zdim = [0, 10];
    pr.ws = [xdim; ydim; zdim];
end
% robot physics
pr.robot_type = 0;                  % robot type
pr.radius = 0.2;                    % robot radius
pr.maxSpeed = 0.4;                  % robot maximal speed
pr.boundLocalRadius = 2.0;          % local bound radius, m
% simulation setup
pr.dtSim = 0.1;                     % time step, s
pr.N = 10;                          % number of stage
pr.ifSimRobotNoise = 1;             % if simulating measurement noise
pr.ifSimBoxObsNoise= 1;
% collision probability threshold
pr.collision_probability = 0.10;
pr.collision_parameter = erfinv(2*sqrt(1-pr.collision_probability) - 1);
% BVC or BUAVC
pr.method = mode_region;            % 0 - BVC; 1 - BUAVC
if pr.ifSimRobotNoise == 0
    pr.method = 0;
    fprintf('Simulate robot localization noise: No. \n');
    fprintf('Computation of safe region: BVC. \n');
else
    fprintf('Simulate robot localization noise: Yes. \n');
    if pr.method == 0
        fprintf('Computation of safe region: BVC. \n');
    elseif pr.method == 1
        fprintf('Computation of safe region: BUAVC. \n');
    else
        error('Safe region mode is not defined!');
    end
end
pr.control = mode_control;          % 0 - one-step; 1 - mpc
if pr.control == 0
    fprintf('Controller mode: Feedback reactive. \n');
elseif pr.control == 1
    fprintf('Controller mode: MPC. \n');
else 
    error('Controller mode is not defined!');
end 
% weights
pr.weights = [0.0, 0.0; 1.0, 0.0]'; % w_pos, w_input, stage and terminal weights

%% Simulation configuration
global cfg
% visualization setup
cfg.ifShowHistoryTra  = 1;          % if plotting the history trajectory of the robot
cfg.ifShowSafeRegion  = 1;          % if plotting the obstacle-free safe region for each robot
cfg.ifShowVelocity    = 0;          % if plotting robot velocity

%% Scenario setup
% static obstacles
if pr.dim == 2
    vert_m = 4;
    [nBoxObs, boxPos, boxSize, boxYaw] = box_initial_2D(3);
elseif pr.dim == 3
    vert_m = 8;
    [nBoxObs, boxPos, boxSize, boxYaw] = box_initial_3D(3);
end
boxVert = zeros(pr.dim, vert_m, nBoxObs);
for iBox = 1 : nBoxObs
    [temp_vert, ~] = box2PolyVertsCons(pr.dim, boxPos(:, iBox), ...
        boxSize(:, iBox), boxYaw(:, iBox));
    boxVert(:, :, iBox) = temp_vert;        % dim * m * nBoxObs
end
% multiple robot, robot initial and end position should not collide with each other and with obstacles
nRobot          = 5;                        % number of robots
collision_mtx   = ones(nRobot, nRobot+nBoxObs);
while (sum(sum(collision_mtx)) > 0)
    fprintf('Generating robot initial positions and goals ... \n');
    if pr.dim == 2
        robotStartPos   = robotStartPos_2D(nRobot, 3, pr.ws(1,:), pr.ws(2,:), pr.radius);
%             robotEndPos     = robotStartPos_2D(nRobot, 2, pr.ws(1,:), pr.ws(2,:), pr.radius);
        robotEndPos     =  -robotStartPos;	% robot final goal position
    elseif pr.dim == 3
        robotStartPos   = robotStartPos_3D(nRobot, 2, pr.ws(1,:), pr.ws(2,:), pr.ws(3,:), pr.radius);
        robotEndPos(1:2, :) = -robotStartPos(1:2, :);   % robot final goal position
        robotEndPos(3, :) = zdim(1) + zdim(2) - robotStartPos(3, :);
    end
    collision_mtx_start = collision_check(nRobot, nBoxObs, robotStartPos, 1.2*pr.radius, boxVert);
    collision_mtx_end   = collision_check(nRobot, nBoxObs, robotEndPos, 1.2*pr.radius, boxVert);
    collision_mtx = [collision_mtx_start; collision_mtx_end];
end
% robot localization measurement noise
robotPosNoise = zeros(pr.dim, pr.dim, nRobot);
for iRobot = 1 : nRobot
    robotPosNoise(:, :, iRobot) = 0.10^2 * eye(pr.dim);
end
% obs localization measurment noise
boxPosNoise = zeros(pr.dim, pr.dim, nBoxObs);
for iBox = 1 : nBoxObs
    boxPosNoise(:, :, iBox) = 0.10^2 * eye(pr.dim);
end
pr.nRobot = nRobot;
pr.robotStartPos = robotStartPos;
pr.robotEndPos = robotEndPos;
pr.robotPosNoise = robotPosNoise;
pr.nBoxObs = nBoxObs;
pr.boxPos = boxPos;
pr.boxSize = boxSize;
pr.boxYaw = boxYaw;
pr.boxPosNoise = boxPosNoise;

%% For mpc
si_mpc_setup;
