%% initialization script
% common pre-run set up for simulation and experiment
% for differential-drive robots

%% Setup path
setPath;


%% Problem setup
global pr
% environment dimension
pr.dim = 2;                         % problem dimension
xdim = [-5, 5];                     % workspace
ydim = [-5, 5];
pr.ws = [xdim; ydim];
% robot physics
pr.robot_type = 1;
pr.radius = 0.2;                    % robot radius
pr.maxSpeed = 0.8;                  % robot maximal speed
pr.maxYawRate = deg2rad(90);        % rad/s
pr.boundLocalRadius = 40.0;         % local bound radius, m
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
end
pr.control = mode_control;          % 0 - one-step; 1 - mpc
pr.simMode = mode_sim;              % 0 - experiments; 1 - simple simulation
pr.expID = 1:100;
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
vert_m = 4;
[nBoxObs, boxPos, boxSize, boxYaw] = box_initial_2D(0);
boxVert_inflate = zeros(pr.dim, vert_m, nBoxObs);
for iBox = 1 : nBoxObs
    [temp_vert, ~] = box2PolyVertsCons(pr.dim, boxPos(:, iBox), ...
        boxSize(:, iBox) + 2*pr.radius, boxYaw(:, iBox));
    boxVert_inflate(:, :, iBox) = temp_vert;        % dim * m * nBoxObs
end
% multiple robot, robot initial and end position should not collide with each other and with obstacles
nRobot          = 5;                        % number of robots
collision_mtx   = ones(nRobot, nRobot+nBoxObs);
while (sum(sum(collision_mtx)) > 0)
    fprintf('Generating robot initial positions and goals ... \n');
    robotStartPos   =   robotStartPos_2D(nRobot, 3, pr.ws(1,:), pr.ws(2,:), pr.radius);
    robotEndPos     =  -robotStartPos;	% robot final goal position
    collision_mtx_start = collision_check(nRobot, nBoxObs, robotStartPos, pr.radius, boxVert_inflate);
    collision_mtx_end   = collision_check(nRobot, nBoxObs, robotEndPos, pr.radius, boxVert_inflate);
    collision_mtx = [collision_mtx_start; collision_mtx_end];
end
% robot localization measurement noise
robotPosNoise = zeros(pr.dim, pr.dim, nRobot);
for iRobot = 1 : nRobot
    robotPosNoise(:, :, iRobot) = 0.06^2 * eye(pr.dim);
end

