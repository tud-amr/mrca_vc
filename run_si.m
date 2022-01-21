% main run for multi-robot collision avoidance using voronoic cell-based
% method, single-integrator robots

% function run_si(varargin)

% if nargin > 0
%     i_test = varargin{1};
% else
%     i_test = 0;
% end

%% clean workspace
close all
clear all
clear
clc


seed_num = 0;
rng(seed_num, 'twister');


%% Choose mode
mode_dim = 2;       % 2: 2D; 3: 3D
mode_region = 1;    % 0: BVC; 1: BUAVC
mode_control = 0;   % 0: reactive; 1: MPC


%% initilization
initialize_si;


%% create a multi-robot system
System = CSystem(nRobot, nBoxObs, pr);


%% robot initial state
for iRobot = 1 : nRobot
    % pos
    System.MultiRobot_{iRobot}.pos_real_ = robotStartPos(:, iRobot);
    System.MultiRobot_{iRobot}.pos_est_  = robotStartPos(:, iRobot);
    % pos cov
    System.MultiRobot_{iRobot}.pos_measure_cov_ = robotPosNoise(:, :, iRobot);
    % goal
    System.MultiRobot_{iRobot}.goal_final_  = robotEndPos(:, iRobot);
    System.MultiRobot_{iRobot}.goal_current_= robotEndPos(:, iRobot);
end


%% box obs initial state
for jBoxObs = 1 : nBoxObs
    % pos
    System.MultiBoxObs_{jBoxObs}.pos_real_ = boxPos(:, jBoxObs);
    System.MultiBoxObs_{jBoxObs}.pos_est_  = boxPos(:, jBoxObs);
    % pos cov
    System.MultiBoxObs_{jBoxObs}.pos_est_cov_ = boxPosNoise(:, :, jBoxObs);
    % size
    System.MultiBoxObs_{jBoxObs}.size_ = boxSize(:, jBoxObs);
    % yaw
    System.MultiBoxObs_{jBoxObs}.yaw_ = boxYaw(:, jBoxObs);
end


%% visulizaton initialization
for i = 1       % for wrapping up
    color_robot = cell(1, nRobot);
    if nRobot <= 7
        color_robot(1, 1:7) = {'r', 'g', 'b', 'm' , 'c', 'k', 'y'};
    else
        color_robot(1, 1:7) = {'r', 'g', 'b', 'm' , 'c', 'k', 'y'};
        for iRobot = 8 : nRobot
            color_robot(1, iRobot) = {rand(1,3)};
        end
    end
    fig_main = figure;                          % main figure
    hold on;
    axis equal
    ax_main = fig_main.CurrentAxes;
    if pr.dim == 2
        axis([pr.ws(1,:), pr.ws(2,:)]);
    elseif pr.dim == 3
        axis([pr.ws(1,:), pr.ws(2,:), pr.ws(3,:)]);
        view(ax_main, 3);
    end
    box on;
    grid on;
    % plot static obstacles
    fig_box_obs = cell(nBoxObs, 1);
    for jBoxObs = 1 : nBoxObs
        fig_box_obs{jBoxObs} = plot_box(ax_main, pr.dim, boxPos(:, jBoxObs), ...
            boxSize(:, jBoxObs), boxYaw(:, jBoxObs), ...
            'FaceColor', [0.4 0.4 0.4], 'FaceAlpha', 0.6, ...
            'EdgeColor', 'k', 'EdgeAlpha', 0.8);
    end
    % plot robot initial positions with figure handles
    fig_robot_pos = cell(nRobot, 1);
    for iRobot = 1 : nRobot
        if pr.dim == 2
            fig_robot_pos{iRobot} = plot_ellipse_2D(ax_main, System.MultiRobot_{iRobot}.pos_real_, ...
                System.MultiRobot_{iRobot}.radius_.*[1;1], 0, ...
                'EdgeColor', color_robot{iRobot}, ...
                'FaceColor', color_robot{iRobot});
        elseif pr.dim == 3
            fig_robot_pos{iRobot} = plot_ellipsoid_3D(ax_main, System.MultiRobot_{iRobot}.pos_real_, ...
                System.MultiRobot_{iRobot}.radius_.*[1;1;1], 0, ...
                'EdgeColor', color_robot{iRobot}, ...
                'FaceColor', color_robot{iRobot});
        end
    end
    % plot robot obstacle-free convex region with figure handles
    fig_robot_region = cell(nRobot, 1);
    if cfg.ifShowSafeRegion
        for iRobot = 1 : nRobot
            pgon = polyshape();     % an empty polygon
            fig_robot_region{iRobot} = plot(ax_main, pgon, 'FaceColor', 'none', 'EdgeColor', ...
                color_robot{iRobot}, 'LineWidth', 1.0, 'LineStyle', ':');
        end
    end
    % plot robot history trajectory with figure handles
    fig_robot_his_tra = cell(nRobot, 1);
    if cfg.ifShowHistoryTra == 1
        robotHistoryTra_x = [];
        robotHistoryTra_y = [];
        robotHistoryTra_z = [];
        for iRobot = 1 : nRobot
            robotHistoryTra_x(1, iRobot) = System.MultiRobot_{iRobot}.pos_real_(1);
            robotHistoryTra_y(1, iRobot) = System.MultiRobot_{iRobot}.pos_real_(2);
            if pr.dim == 2
                fig_robot_his_tra{iRobot} = plot(robotHistoryTra_x(:, iRobot), ...
                    robotHistoryTra_y(:, iRobot), 'Color', color_robot{iRobot}, ...
                    'LineWidth', 1.5, 'LineStyle', '-');
            elseif pr.dim == 3
                robotHistoryTra_z(1, iRobot) = System.MultiRobot_{iRobot}.pos_real_(3);
                fig_robot_his_tra{iRobot} = plot3(robotHistoryTra_x(:, iRobot), ...
                    robotHistoryTra_y(:, iRobot), robotHistoryTra_z(:, iRobot), ...
                    'Color', color_robot{iRobot}, ...
                    'LineWidth', 1.5, 'LineStyle', '-');
            end
        end
    end
end


%% data logging
logsize = 500;
% robot
log_robot_goal_final   = zeros(pr.dim, logsize, nRobot);
log_robot_goal_current = zeros(pr.dim, logsize, nRobot);
log_robot_pos_real  = zeros(pr.dim, logsize, nRobot);
log_robot_pos_est   = zeros(pr.dim, logsize, nRobot);
log_robot_arrived   = zeros(1, logsize, nRobot);
log_robot_collision = zeros(1, logsize, nRobot);
log_robot_input     = zeros(pr.dim, logsize, nRobot);
% obs
log_obs_pos_real    = zeros(pr.dim, logsize, nBoxObs);
log_obs_pos_est     = zeros(pr.dim, logsize, nBoxObs);
log_obs_size        = zeros(pr.dim, logsize, nBoxObs);
log_obs_yaw         = zeros(1, logsize, nBoxObs);


%% loop preparation
iRobot              =   0;              % clear index
jBoxObs             =   0;
n_loop              =   0;              % number of loops performed
exitflag            =   0;              % flags
infeasible          =   0;
fprintf('[%s] MODE: Running as Simulation \n',datestr(now,'HH:MM:SS'));
fprintf('[%s] Looping... \n',datestr(now,'HH:MM:SS'));


%% main loop
if_robots_arrived = zeros(nRobot, 1);
% pause;
while(true && n_loop < logsize)
    
    %% control loop
    n_loop = n_loop + 1;
    if(mod(n_loop, 10) == 0)
        fprintf('[%s] Looping [%d] \n',datestr(now,'HH:MM:SS'), n_loop);
    end
     
    %% get system state
    System.getSystemState();
    
    %% logging state info
    for iRobot = 1 : nRobot
        log_robot_pos_real(:, n_loop, iRobot) = ...
            System.MultiRobot_{iRobot}.pos_real_;
        log_robot_pos_est(:, n_loop, iRobot) = ...
            System.MultiRobot_{iRobot}.pos_est_;
    end
    for jBox = 1 : nBoxObs
        log_obs_pos_real(:, n_loop, jBox) = System.multi_obs_pos_real_(:, jBox);
        log_obs_pos_est(:, n_loop, jBox) = System.multi_obs_pos_est_(:, jBox);
        log_obs_size(:, n_loop, jBox) = System.multi_obs_size_real_(:, jBox);
        log_obs_yaw(:, n_loop, jBox) = System.multi_obs_yaw_real_(:, jBox);
    end
    
    %% update system visulization
    for i = 1       % for wrapping up
        % robot current position
        for iRobot = 1 : nRobot
            if pr.dim == 2
                [X, Y] = ellipse(System.MultiRobot_{iRobot}.pos_real_, ...
                    System.MultiRobot_{iRobot}.radius_.*[1;1], 0);
                set(fig_robot_pos{iRobot}, 'XData', X, 'YData', Y); 
            elseif pr.dim == 3
                [X, Y, Z] = ellipsoid(System.MultiRobot_{iRobot}.pos_real_(1), ...
                    System.MultiRobot_{iRobot}.pos_real_(2), ...
                    System.MultiRobot_{iRobot}.pos_real_(3), ...
                    System.MultiRobot_{iRobot}.radius_, ...
                    System.MultiRobot_{iRobot}.radius_, ...
                    System.MultiRobot_{iRobot}.radius_);
                set(fig_robot_pos{iRobot}, 'XData', X, 'YData', Y, 'ZData', Z); 
            end
        end
        % robot convex region
        if cfg.ifShowSafeRegion == 1
            for iRobot = 1 : nRobot
                delete(fig_robot_region{iRobot});
                fig_robot_region{iRobot} = plot_poly_vert(ax_main, pr.dim, ...
                    System.MultiRobot_{iRobot}.safe_region_.verts', ...
                    'FaceColor', color_robot{iRobot}, 'FaceAlpha', 0.1, ...
                    'EdgeColor', color_robot{iRobot}, 'EdgeAlpha', 0.6, ...
                    'LineWidth', 1.0, 'LineStyle', '-.');
            end
        end
        % robot history trajectory
        if cfg.ifShowHistoryTra == 1
            robotHistoryTra_x(end+1, 1) = 0;        % extend the array
            robotHistoryTra_y(end+1, 1) = 0;
            robotHistoryTra_z(end+1, 1) = 0;
            for iRobot = 1 : nRobot
                robotHistoryTra_x(end, iRobot) = System.MultiRobot_{iRobot}.pos_real_(1);
                robotHistoryTra_y(end, iRobot) = System.MultiRobot_{iRobot}.pos_real_(2);
                if pr.dim == 2
                    set(fig_robot_his_tra{iRobot}, 'XData', robotHistoryTra_x(:, iRobot), ...
                        'YData', robotHistoryTra_y(:, iRobot));
                elseif pr.dim == 3
                    robotHistoryTra_z(end, iRobot) = System.MultiRobot_{iRobot}.pos_real_(3);
                    set(fig_robot_his_tra{iRobot}, 'XData', robotHistoryTra_x(:, iRobot), ...
                        'YData', robotHistoryTra_y(:, iRobot), ...
                        'ZData', robotHistoryTra_z(:, iRobot));
                end
            end
        end
    end
    
    if sum(sum(System.collision_mtx_)) > 0
        fprintf('Collision happens!\n')
        pause;
%         break;
    end
    
    %% simulate one step
    System.simSystemOneStep();
    
    %% if robot arrived
    for iRobot = 1 : nRobot
        if_robots_arrived(iRobot) = System.MultiRobot_{iRobot}.isArrived_;
    end
    
    %% collision checking
    System.collisionChecking();    
    
    %% logging control info
    for iRobot = 1 : nRobot
        log_robot_goal_final(:, n_loop, iRobot) = ...
            System.MultiRobot_{iRobot}.goal_final_;
        log_robot_goal_current(:, n_loop, iRobot) = ...
            System.MultiRobot_{iRobot}.goal_current_;
        log_robot_arrived(:, n_loop, iRobot) = ...
            System.MultiRobot_{iRobot}.isArrived_;
        log_robot_collision(:, n_loop, iRobot) = ...
            System.MultiRobot_{iRobot}.isCollision_;
        log_robot_input(:, n_loop, iRobot) = ...
            System.MultiRobot_{iRobot}.u_;
    end
    
    
    drawnow limitrate
    pause(0.01);
%     pause(pr.dtSim);
    
    %% end of simulation
    if sum(if_robots_arrived) == nRobot
        fprintf('All robots arrived!\n');
%         pause;
        break;
    end
    
end
