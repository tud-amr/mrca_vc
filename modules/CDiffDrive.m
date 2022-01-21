classdef CDiffDrive < handle
    % Class for single-integrator robots
    
    properties
       
        %% dimension, id, indicator
        dim_                =   0;                  % 2 - two-dimensional
        id_                 =   0;                  % index of the robot
        exp_id_             =   0;                  % for exp
        
        simMode_            =   0;                  % sim or exp
        robot_type_         =   1;
        
        %% radius
        radius_             =   0;                  % r
        
        %% time step
        dt_                 =   0;                  % s
        N_                  =   0;                  % number of stage of prediction window
        
        %% maximal speed and yaw rate
        speed_max_          =   0;         
        yaw_rate_max_       =   0;
        
        %% global goal and projected goal
        goal_final_         =   [];                 % global final goal
        goal_current_       =   [];                 % local current goal
        goal_projected_     =   [];                 % projected goal in free region
        goal_tolerance_     =   0;
        
        %% workspace bounds
        bound_global_       =   [];                 % global workspace bound, dim*2, [min, max]
        bound_local_        =   [];                 % local bound, represented in global coordinate
        bound_radius_       =   [];                 % local bound radius
        
        %% workspace information
        % other robot
        robots_info_local_  =   {};                 % local other robots info
        obs_info_local_     =   {};                 % local obs info
        
        %% real state
        pos_real_           =   [];                 % x, y
        yaw_real_           =   [];                 % theta
        
        %% measured state
        pos_measure_        =   [];
        pos_measure_cov_    =   [];                 % position measurement noise
        yaw_measure_        =   0;
        
        %% estimated state
        pos_est_            =   [];
        pos_est_cov_        =   [];                 % position estimation noise
        yaw_est_            =   0;
        
        %% control input
        u_              =   [];                     % [v, omega]
        
        %% uncertainty information
        if_sim_noise_       =   0;                  % if simulate the measurement noise
        
        %% deadlock resolving
        isCollision_        =   0;                  % if in collision
        isArrived_          =   0;                  % if has arrived the final goal
        isDeadlock_         =   0;                  % if in a deadlock situation
        isAtVertex_         =   0;                  % if at some vertex of the convex region
        isAtEdge_           =   0;                  % if at some edge
        infoAtEdge_         =   {};                 % info of the edge
        
        ifAvoidRobotDeadlock_=  0;                  % if in avoiding robot deadlock
        numStepRotationGoal_=   1;                  % required steps for moving to rotation goal
        kStepRotationGoal_  =   0;                  % how long has keeping current rotation goal
        
        ifAvoidObsDeadlock_ =   0;                  % if in avoiding obs deadlock
        ifReachCurrentGoal_ =   0;                  % if reaching current goal, for obs deadlock avoiding
        
        %% local obstacle-free convex region information
        BVC_                =   {};                 % bvc
        BUAVC_              =   {};                 % buavc
        safe_region_        =   {};                 % safe region
        
        %% collision probability
        collision_probability_ = 0;
        collision_parameter_   =  0;
        
        %% for mpc
        mpc_weights_        =   [];                 % w_wp_pos, w_input_v
        mpc_exitflag_                               % MPC solving exitflag
        mpc_info_                                   % MPC solving information
        mpc_Xk_             = [];                   % store initial conditions for MPC
        mpc_Zk_             = [];                   % store computed current stage from MPC
        mpc_Zk2_            = [];                   % store computed next stage from MPC
        mpc_ZPlan_          = [];                   % store entire planned MPC plan
        mpc_Path_           = [];                   % store MPC planned path
        
        %% ROS
        pose_measure_sub_                           % subscribe to mocap raw data
        cmd_vel_pub_                                % publish computed control input
        
    end
    
    
    methods
        
        %% ========== Constructor ==========
        function obj = CDiffDrive(pr, robotID)
            
            obj.id_     =   robotID;
            obj.exp_id_ =   pr.expID(robotID);
            
            obj.simMode_=   pr.simMode;
            
            obj.dim_    =   pr.dim;
            obj.dt_     =   pr.dtSim;
            obj.N_      =   pr.N;
            obj.radius_ =   pr.radius;
            obj.speed_max_=   pr.maxSpeed;
            obj.yaw_rate_max_ = pr.maxYawRate;
            obj.if_sim_noise_ = pr.ifSimRobotNoise;
            
            obj.mpc_weights_ = pr.weights;  % stage and terminal weights
            
            obj.collision_probability_ = pr.collision_probability;
            obj.collision_parameter_   = pr.collision_parameter;
            
            obj.bound_radius_ = pr.boundLocalRadius;
            obj.bound_global_ = pr.ws;      % dim*2
            obj.bound_local_  = pr.ws;      % dim*2
            
            % vector and matrix initialization
            obj.pos_real_   =   zeros(obj.dim_, 1);
            obj.pos_measure_=   zeros(obj.dim_, 1);
            obj.pos_est_    =   zeros(obj.dim_, 1);
            obj.pos_measure_cov_    =   0.01^2 * eye(obj.dim_);
            obj.pos_est_cov_        =   0.01^2 * eye(obj.dim_);
            
            obj.u_          =   zeros(2, 1);
            
            obj.goal_final_ =   zeros(obj.dim_, 1);
            obj.goal_current_=  zeros(obj.dim_, 1);
            obj.goal_projected_     =   zeros(obj.dim_, 1);
            
            obj.robots_info_local_.idx      = [];
            obj.robots_info_local_.pos      = [];
            obj.robots_info_local_.pos_cov  = [];
            
            obj.obs_info_local_.idx         = [];
            obj.obs_info_local_.pos         = [];
            obj.obs_info_local_.pos_cov     = [];
            obj.obs_info_local_.size        = [];
            obj.obs_info_local_.yaw         = [];
            
            obj.BVC_.Ab_bound   =   [];     
            obj.BVC_.idx_robot  =   [];     % m*1
            obj.BVC_.Ab_robot	=   [];     % m*(dim+1)
            obj.BVC_.idx_obs    =   [];
            obj.BVC_.Ab_robot   =   [];
            
            obj.BVC_.Ab         =   [];
            obj.BVC_.verts      =   [];     % p*dim
            
            obj.BUAVC_.Ab_bound =   [];
            obj.BUAVC_.idx_robot=   [];
            obj.BUAVC_.Ab_robot	=   [];
            obj.BUAVC_.idx_obs  =   [];
            obj.BUAVC_.Ab_robot =   [];
            
            obj.BUAVC_.Ab       =   [];
            obj.BUAVC_.verts    =   [];
            
            % initial safe region
            [A_bound, b_bound] = boundsCon(obj.dim_, pr.ws(:,1), pr.ws(:,2));
            obj.safe_region_.Ab  = [A_bound, b_bound];
            obj.safe_region_.verts = lcon2vert(obj.safe_region_.Ab(:,1:obj.dim_), ...
                obj.safe_region_.Ab(:,obj.dim_+1));
            
            % at edge info
            obj.infoAtEdge_.idx_robot = 0;
            obj.infoAtEdge_.idx_obs   = 0;
            
            % tolerance for reaching the goal
            if obj.if_sim_noise_ == 1
                obj.goal_tolerance_ = 0.2;%0.5;                   % if simulating noise
            else
                obj.goal_tolerance_ = 0.2;%4*obj.dt_ * obj.speed_max_; % in deterministic case
            end
            
        end
        
        
        %%  =======================================================================
        function initializeROS(obj)
            % Initialize ROS publishers and subscribers for the quadrotor
            
            expID = obj.exp_id_;
                       
            %% ROS subscribers
            % mocap raw data
            obj.pose_measure_sub_ = rossubscriber(...
                ['/Bebop', num2str(expID), '/pose'], 'geometry_msgs/PoseStamped');
            
            %% ROS publisher
            % to publish mpc control input
            if obj.exp_id_ ~= 1
                obj.cmd_vel_pub_ = rospublisher(... % '/jackal_velocity_controller/cmd_vel', ...
                    ['/robot', num2str(obj.exp_id_), '/cmd_vel'], ...
                    'geometry_msgs/Twist', 'IsLatching', false);
            else
                obj.cmd_vel_pub_ = rospublisher(... % '/jackal_velocity_controller/cmd_vel', ...
                    '/cmd_vel', ...
                    'geometry_msgs/Twist', 'IsLatching', false);
            end

        end
        
        
        
        %% ========== Get measured state ==========
        function getMeasuredState(obj)
            
            if obj.simMode_ == 0        % in experiments
                % geometry_msgs/PoseStamped
                msg = obj.pose_measure_sub_.LatestMessage; 
                if ~isempty(msg)
                    % measured position [x y z]
                    pos_measure = [msg.Pose.Position.X;
                                   msg.Pose.Position.Y;
                                   msg.Pose.Position.Z];
                    % measured quaternion [w x y z]
                    q_measure = [ msg.Pose.Orientation.W;
                                  msg.Pose.Orientation.X;
                                  msg.Pose.Orientation.Y;
                                  msg.Pose.Orientation.Z];
                    % transform to euler angles (RPY, roll, pitch, yaw)
                    euler_measure = quat2eul(q_measure', 'XYZ')';

                    % read as "real" data
                    obj.pos_real_ = [pos_measure(1); pos_measure(2)];
                    obj.yaw_real_ = euler_measure(3);
                else
                     warning('Robot %i is not tracked! \n', obj.id_);
                end
            end
            
            % first, same as the real state
            obj.pos_measure_ = obj.pos_real_;
            obj.yaw_measure_ = obj.yaw_real_;

            % then add measurement noise (Gaussian) to generate observed state if required
            if obj.if_sim_noise_ == 1
                dpos = zeros(obj.dim_, 1);
                for i = 1 : obj.dim_
                    dpos(i) = random('Normal', 0, sqrt(obj.pos_measure_cov_(i,i)));
                end
                obj.pos_measure_ = obj.pos_measure_ + dpos;
            end                    
            
        end
        
        
        %% ========== Get estimated state ==========
        function getEstimatedState(obj)
            
            % first get the measured state
            obj.getMeasuredState();
            
            % simply make the estimated state equal to the measured
            obj.pos_est_ = obj.pos_measure_;
            obj.pos_est_cov_ = obj.pos_measure_cov_;
            obj.yaw_est_ = obj.yaw_measure_;
            
        end
        
        
        %% ========== Initialize mpc ==========
        function initializeMPC(obj, x_start, mpc_plan)
            
            global index
            
            obj.mpc_Xk_    = x_start;
            obj.mpc_ZPlan_ = mpc_plan;
            obj.mpc_Path_  = mpc_plan(index.z.pos, :);
            
        end
        
        
        %% ========== Compute local bound ==========
        function computeLocalBound(obj)
            
            for i = 1 : obj.dim_
                lb = obj.pos_est_(i) - obj.bound_radius_;
                ub = obj.pos_est_(i) + obj.bound_radius_;
                
                obj.bound_local_(i, 1) = max(lb, obj.bound_global_(i, 1));
                obj.bound_local_(i, 2) = min(ub, obj.bound_global_(i, 2));
            end
            
        end
        
        
        %% ========== If an object in local bound ==========
        function inside = isPointInLocalBound(obj, pt)
            
            inside = 1;
            for i = 1 : obj.dim_
                if pt(i) < obj.bound_local_(i, 1) ...
                        || pt(i) > obj.bound_local_(i, 2)
                    inside = 0;
                end
            end
            
        end
        
        
        %% ========== Get local robots info ==========
        function getLocalRobotInfo(obj, multi_robot_pos_est, multi_robot_pos_est_cov)
            
            obj.robots_info_local_.idx      = [];
            obj.robots_info_local_.pos      = [];
            obj.robots_info_local_.pos_cov  = [];
            
            nRobot = size(multi_robot_pos_est, 2);
            k = 0;
            for iRobot = 1 : nRobot
                if iRobot == obj.id_
                    continue;           % ignore ego robot
                end
                pt = multi_robot_pos_est(:, iRobot);
                inside = obj.isPointInLocalBound(pt);
                if inside               % only count inside
                    k = k + 1;
                    obj.robots_info_local_.idx(k) = iRobot;
                    obj.robots_info_local_.pos(:,k) = multi_robot_pos_est(:,iRobot);
                    obj.robots_info_local_.pos_cov(:,:,k) = multi_robot_pos_est_cov(:,:,iRobot);
                end
            end
            
        end
        
        
        %% ========== Get local obs info ==========
        function getLocalObsInfo(obj, multi_obs_pos_est, multi_obs_pos_est_cov, ...
                multi_obs_size, multi_obs_yaw)
            
            obj.obs_info_local_.idx         = [];
            obj.obs_info_local_.pos         = [];
            obj.obs_info_local_.pos_cov     = [];
            obj.obs_info_local_.size        = [];
            obj.obs_info_local_.yaw         = [];
            
            nBoxObs = size(multi_obs_pos_est, 2);
            k = 0;
            for jBoxObs = 1 : nBoxObs
                pt = multi_obs_pos_est(:, jBoxObs);
                inside = obj.isPointInLocalBound(pt);
                if inside               % only count inside
                    k = k + 1;
                    obj.obs_info_local_.idx(k) = jBoxObs;
                    obj.obs_info_local_.pos(:,k) = multi_obs_pos_est(:,jBoxObs);
                    obj.obs_info_local_.pos_cov(:,:,k) = multi_obs_pos_est_cov(:,:,jBoxObs);
                    obj.obs_info_local_.size(:,k) = multi_obs_size(:,jBoxObs);
                    obj.obs_info_local_.yaw(:,k) = multi_obs_yaw(:,jBoxObs);
                end
            end
            
        end
        
        
        %% ========== Compute BVC ==========
        function computeBVC(obj)
            
            obj.BVC_.Ab_bound   =   [];
            
            obj.BVC_.idx_robot  =   [];
            obj.BVC_.Ab_robot	=   [];
            
            obj.BVC_.idx_obs    =   [];
            obj.BVC_.Ab_obs     =   [];
            
            obj.BVC_.Ab         =   [];
            obj.BVC_.verts      =   [];
            
            pe = obj.pos_est_;      % ego position
            r  = obj.radius_;       % ego radius
            
            % local bound
            [A_bound, b_bound] = boundsCon(obj.dim_, ...
                obj.bound_local_(:,1), obj.bound_local_(:,2));
            for k = 1 : size(A_bound, 1)
                obj.BVC_.Ab_bound(k, 1:obj.dim_) = A_bound(k, 1:obj.dim_);
                obj.BVC_.Ab_bound(k, obj.dim_+1) = b_bound(k) - r*norm(A_bound(k, 1:obj.dim_));
            end
            
            % loop for each other local robot
            for i = 1 : size(obj.robots_info_local_.pos, 2)
                pt = obj.robots_info_local_.pos(:, i);  % other robot pos
                [a, b] = point_point_hyperplane(pe, pt);
                obj.BVC_.idx_robot(i) = obj.robots_info_local_.idx(i);
                obj.BVC_.Ab_robot(i, 1:obj.dim_) = a';
                obj.BVC_.Ab_robot(i, obj.dim_+1) = b - r*norm(a');
            end
            
            % loop for each local obs
            for i = 1 : size(obj.obs_info_local_.pos, 2)
                obs_pt = obj.obs_info_local_.pos(:, i);     % obs pos
                obs_size = obj.obs_info_local_.size(:, i);  % obs size
                obs_yaw = obj.obs_info_local_.yaw(:, i);    % obs yaw
                [a, b] = point_box_shifted_hyperplane(pe, obs_pt, obs_size, obs_yaw);
                obj.BVC_.idx_obs(i) = obj.obs_info_local_.idx(i);
                obj.BVC_.Ab_obs(i, 1:obj.dim_) = a';
                obj.BVC_.Ab_obs(i, obj.dim_+1) = b - r*norm(a');
            end
            
            % combine Ab
            obj.BVC_.Ab = [obj.BVC_.Ab_bound; obj.BVC_.Ab_robot; obj.BVC_.Ab_obs];
            
            % convert to verts
            obj.BVC_.verts = qlcon2vert(pe, obj.BVC_.Ab(:,1:obj.dim_), ...
                obj.BVC_.Ab(:,obj.dim_+1));
            
            obj.safe_region_ = obj.BVC_;
            
        end
        
        
        %% ========== Compute BUAVC ==========
        function computeBUAVC(obj)
            
            obj.BUAVC_.Ab_bound     =   [];
            
            obj.BUAVC_.idx_robot    =   [];
            obj.BUAVC_.Ab_robot     =   [];
            
            obj.BUAVC_.idx_obs      =   [];
            obj.BUAVC_.Ab_obs       =   [];
            
            obj.BUAVC_.Ab           =   [];
            obj.BUAVC_.verts        =   [];
            
            pe = obj.pos_est_;              % ego robot position
            pe_cov = obj.pos_est_cov_;      % ego robot pos covariance
            pr = obj.collision_parameter_;  % for computing buffer
            r = obj.radius_;                % ego robot radius
            
            % local bound
            [A_bound, b_bound] = boundsCon(obj.dim_, ...
                obj.bound_local_(:,1), obj.bound_local_(:,2));
            for k = 1 : size(A_bound, 1)
                obj.BUAVC_.Ab_bound(k, 1:obj.dim_) = A_bound(k, 1:obj.dim_);
                a = A_bound(k, 1:obj.dim_)';
                buffer_r = r*norm(a);
                buffer_u = pr * sqrt(2*a'*pe_cov*a);
                obj.BUAVC_.Ab_bound(k, obj.dim_+1) = b_bound(k) - buffer_r - buffer_u;
            end
            
            % loop for each other local robot
            for i = 1 : size(obj.robots_info_local_.pos, 2)
                pt = obj.robots_info_local_.pos(:, i);  % other robot pos
                pt_cov = obj.robots_info_local_.pos_cov(:,:,i); % other robot pos cov
                [a, b] = gaussian_gaussian_hyperplane(pe, pe_cov, pt, pt_cov);
                obj.BUAVC_.idx_robot(i) = obj.robots_info_local_.idx(i);
                obj.BUAVC_.Ab_robot(i, 1:obj.dim_) = a';
                buffer_r = r*norm(a);
                buffer_u = pr * sqrt(2*a'*pe_cov*a);
                obj.BUAVC_.Ab_robot(i, obj.dim_+1) = b - buffer_r - buffer_u;
            end
            
            % loop for each local obs
            for i = 1 : size(obj.obs_info_local_.pos, 2)
                obs_pt = obj.obs_info_local_.pos(:, i);     % obs pos
                obs_pt_cov = obj.obs_info_local_.pos_cov(:,:,i); % obs pos cov
                obs_size = obj.obs_info_local_.size(:, i);  % obs size
                obs_yaw = obj.obs_info_local_.yaw(:, i);    % obs yaw
                [a, b] = point_box_gaussian_shifted_hyperplane_simple(pe, ...
                    obs_pt, obs_pt_cov, obs_size, obs_yaw, obj.collision_probability_);
                obj.BUAVC_.idx_obs(i) = obj.obs_info_local_.idx(i);
                obj.BUAVC_.Ab_obs(i, 1:obj.dim_) = a';
                buffer_r = r*norm(a);
                buffer_u = pr * sqrt(2*a'*pe_cov*a);
                obj.BUAVC_.Ab_obs(i, obj.dim_+1) = b - buffer_r - buffer_u;
            end
            
            % combine Ab
            obj.BUAVC_.Ab = [obj.BUAVC_.Ab_bound; obj.BUAVC_.Ab_robot; obj.BUAVC_.Ab_obs];
            
            % convert to verts
            obj.BUAVC_.verts = qlcon2vert(pe, obj.BUAVC_.Ab(:,1:obj.dim_), ...
                obj.BUAVC_.Ab(:,obj.dim_+1));
            
            obj.safe_region_ = obj.BUAVC_;
            
        end
        
        
        %% ========== Compute control input ==========
        function computeControlInput(obj)
            
            global pr
            maxYawRate = pr.maxYawRate;
            
            obj.u_ = zeros(obj.dim_, 1);
            
            %% compute projected goal
            % project to safe region
            [obj.goal_projected_, exitflag] = project_to_poly(obj.goal_current_, ...
                obj.safe_region_.Ab(:,1:obj.dim_), obj.safe_region_.Ab(:,obj.dim_+1));
            % project to safe region with motion direction
            pos_x = obj.pos_est_(1);
            pos_y = obj.pos_est_(2);
            theta = obj.yaw_est_;
            Aeq_v = [tan(theta), -1];
            beq_v = pos_x*tan(theta) - pos_y;
            [goal_projected_v, exitflag_v] = project_to_poly_line(obj.goal_current_, ...
                obj.safe_region_.Ab(:,1:obj.dim_), obj.safe_region_.Ab(:,obj.dim_+1), ...
                Aeq_v, beq_v);
            % project to safe region with goal direction
            pos_x_g = obj.goal_current_(1);
            pos_y_g = obj.goal_current_(2);
            Aeq_omega = [pos_y - pos_y_g, pos_x_g - pos_x];
            beq_omega = pos_x_g*pos_y - pos_x*pos_y_g;
            [goal_projected_omega, exitflag_omega] = project_to_poly_line(obj.goal_current_, ...
                obj.safe_region_.Ab(:,1:obj.dim_), obj.safe_region_.Ab(:,obj.dim_+1), ...
                Aeq_omega, beq_omega);
            % control input
            if exitflag == 1 && exitflag_v == 1 && exitflag_omega == 1  % feasible
                distance_to_projected_goal = norm(obj.goal_projected_ - obj.pos_est_);
                if distance_to_projected_goal <= obj.goal_tolerance_
                    obj.u_ = zeros(obj.dim_, 1);            % set to zero if reaching the goal
                else
                    % gain
%                     k = obj.speed_max_;
%                     k = 1 / norm(obj.pos_est_ - goal_projected_v);
%                     k = 0.5;
                    v_temp = [cos(theta), sin(theta)] * (obj.pos_est_ - goal_projected_v);
                    omega_temp = atan2([-sin(theta), cos(theta)] * (obj.pos_est_ - 0.5*(goal_projected_omega+obj.goal_projected_)), ...
                        [cos(theta), sin(theta)] * (obj.pos_est_ - 0.5*(goal_projected_omega+obj.goal_projected_)));
                    k_v = abs(obj.speed_max_ / max(abs(v_temp), 0.0001));
                    k_omega = abs(maxYawRate / max(abs(omega_temp), 0.00001));
                    k = 0.99*min(k_v, k_omega);
                    % v
                    obj.u_(1) = -k * [cos(theta), sin(theta)] * ...
                        (obj.pos_est_ - goal_projected_v);
                    % omega
                    obj.u_(2) = -0.5*k * atan2([-sin(theta), cos(theta)] * (obj.pos_est_ - 0.5*(goal_projected_omega+obj.goal_projected_)), ...
                        [cos(theta), sin(theta)] * (obj.pos_est_ - 0.5*(goal_projected_omega+obj.goal_projected_)));
                end  
            else
                obj.u_ = 0 * obj.u_;
            end
            
        end
        
        
        %% ========== Compute control input mpc ==========
        function computeControlInputMPC(obj)
            
            % Fisrst set the real-time parameter vector
            % pAll include parameters for all N stage
            
            global index model
            
            %% prepare parameters
            startPos     = obj.pos_est_;
            wayPoint     = obj.goal_current_;
            dt           = obj.dt_;
            speedMax     = obj.speed_max_;              % speed limit
            weightStage  = obj.mpc_weights_(:, 1);      % stage weights
            weightN      = obj.mpc_weights_(:, 2);      % terminal weights
            hyperplanes  = obj.safe_region_.Ab;         % m * (dim+1)
            m = size(hyperplanes, 1);                   % number of hyperplanes
            assert(m <= model.nHyperplanes, 'Parameter setting error!');
            
            %% all stage parameters
            pStage = zeros(model.npar, 1);              % some stage
            pAll = repmat(pStage, [obj.N_, 1]);         % all stage
            for iStage = 1 : obj.N_
                % general parameter
                pStage(index.p.startPos) = startPos; 	% start position with yaw
                pStage(index.p.wayPoint) = wayPoint;  	% waypoint
                pStage(index.p.dt)       = dt;          % dt
                pStage(index.p.speedLimit) = speedMax;  % maximal speed
                pStage(index.p.weights)  = weightStage; % stage cost weights
                % safe region hyperplane parameters
                idx = 1;
                for iPlane = 1 : model.nHyperplanes
                    if idx <= m
                        pStage(index.p.safeRegion(:, idx)) = hyperplanes(idx, :)';
                    else
                        pStage(index.p.safeRegion(:, idx)) = hyperplanes(m, :)';
                    end
                    idx = idx + 1;
                end
                % change the last stage cost term weights
                if iStage == obj.N_ 
                    pStage(index.p.weights)  = weightN;
                end
                % insert into the all stage parameter
                pAll((1 + model.npar*(iStage-1)) : ...
                    (model.npar + model.npar*(iStage-1))) = pStage;
            end
            
            %% Then call the solver
            problem.all_parameters = pAll;
            % set initial conditions
            obj.mpc_Xk_ = obj.pos_est_;
            problem.xinit = obj.mpc_Xk_;
            % prepare initial guess
            if obj.mpc_exitflag_ == 1    % last step mpc feasible
               x0_temp = reshape([obj.mpc_ZPlan_(:, 2:obj.N_), ...
                   obj.mpc_ZPlan_(:, obj.N_)], obj.N_*model.nvar, 1);
            else                         % last step mpc infeasible
               x0_temp_stage = zeros(model.nvar, 1);
               x0_temp_stage(index.z.pos) = obj.mpc_Xk_;
               x0_temp = repmat(x0_temp_stage, obj.N_, 1);
            end           
            problem.x0 = x0_temp;
            % call the NLP solver
            func_name = strcat('si_', num2str(obj.dim_), 'D', '_', ...
               num2str(model.nHyperplanes), '_', ...
               num2str(obj.N_), '_', num2str(1000*obj.dt_));
            NLPSolver = str2func(func_name);
            [output, exitflag, info] = NLPSolver(problem);

            % store mpc sovling information
            obj.mpc_exitflag_ = exitflag;
            obj.mpc_info_    = info;

            % store output
            for iStage = 1 : obj.N_
               obj.mpc_ZPlan_(:, iStage) = output.(['x', sprintf('%02d', iStage)]);
               obj.mpc_Path_(:, iStage)  = obj.mpc_ZPlan_(index.z.pos, iStage);
            end
            obj.mpc_Zk_  = obj.mpc_ZPlan_(:, 1);
            obj.mpc_Zk2_ = obj.mpc_ZPlan_(:, 2);

            % check the exitflag and get optimal control input
            if exitflag == 0                 % solver reaching maximum iterations
               warning('MPC: Max iterations reached!');
            elseif exitflag == -4
               warning('MPC: Wrong number of inequalities input to solver!');
            elseif exitflag == -5
               warning('MPC: Error occured during matrix factorization!');
            elseif exitflag == -6
               warning('MPC: NaN or INF occured during functions evaluations!');
            elseif exitflag == -7
               warning('MPC: Infeasible! The solver could not proceed!');
            elseif exitflag == -10
               warning('MPC: NaN or INF occured during evaluation of functions and derivatives!');
            elseif exitflag == -11
               warning('MPC: Invalid values in problem parameters!');
            elseif exitflag == -100
               warning('MPC: License error!');
            end

            if exitflag == 1
               % if mpc solved successfully
               obj.u_ = obj.mpc_Zk_(index.z.inputs);
            else
               % if infeasible
                obj.u_ = -0.0 * obj.u_;
            end
            
        end
        
        
        %% ========== Simulate one step ==========
        function simulateOneStep(obj)
            
            % Send and execute the control command
            
            switch obj.simMode_
                case 1                      % in simple simulation mode
                    % simulate one step in simple simulation mode
                    % current state and control
                    xNow = [obj.pos_real_; obj.yaw_real_];
                    u    = obj.u_;      

                    % integrate one step
                    xNext = RK2( xNow, u, @diff_drive_continuous_dynamics, ...
                        obj.dt_, [], 1);

                    % update the implicit real state
                    obj.pos_real_   = xNext(1:2);
                    obj.yaw_real_   = xNext(3);
                    
                otherwise
%                     if obj.exp_id_ == 1
                        % publish control command sent to the Jackal
                        cmd_vel_msg = rosmessage('geometry_msgs/Twist');
                        cmd_vel_msg.Linear.X = obj.u_(1);
                        cmd_vel_msg.Angular.Z = obj.u_(2);
                        obj.cmd_vel_pub_.send(cmd_vel_msg);
%                     end
                    
            end
        end
        
        
        %% ========== Running state checking ==========
        function deadlockChecking(obj)
            
            obj.isArrived_   = 0;
            obj.isDeadlock_  = 0;
            obj.isAtVertex_  = 0;
            obj.isAtEdge_    = 0;
            
            obj.infoAtEdge_.idx_robot = 0;
            obj.infoAtEdge_.idx_obs   = 0;
            
            % ----- check if arrived at final goal -----
            distance_to_final_goal = norm(obj.pos_est_ - obj.goal_final_);
            if distance_to_final_goal <= obj.goal_tolerance_
                obj.isArrived_ = 1;             % arrived
                obj.ifAvoidObsDeadlock_ = 0;    % end the obs deadlock avoiding
                obj.ifAvoidRobotDeadlock_ = 0;  % also end the robot deadlock avoiding
                obj.kStepRotationGoal_ = 0;
                obj.numStepRotationGoal_ = 1;
            end
            
            % ----- check if arrived at current goal -----
            distance_to_current_goal = norm(obj.pos_est_ - obj.goal_current_);
            if distance_to_current_goal <= obj.goal_tolerance_
                obj.ifReachCurrentGoal_ = 1;    % reaching current goal
                obj.ifAvoidObsDeadlock_ = 0;    % end the obs deadlock avoiding
                obj.ifAvoidRobotDeadlock_ = 0;  % also end the robot deadlock avoiding
                obj.kStepRotationGoal_ = 0;
                obj.numStepRotationGoal_ = 1;
            else
                obj.ifReachCurrentGoal_ = 0;
            end
            
            % ----- simply check if in deadlock -----
            % if not reaching final goal but reaching projected goal
            distance_to_projected_goal = norm(obj.pos_est_ - obj.goal_projected_);
            if ~obj.isArrived_ && distance_to_projected_goal <= obj.goal_tolerance_
                obj.isDeadlock_ = 1;
            end
            % if not reaching final goal but the velocity is zero
            if ~obj.isArrived_ && norm(obj.u_) < 0.1*obj.speed_max_ && norm(obj.u_) > 0.00001
                obj.isDeadlock_ = 1;
            end
            
            % ----- check if at vertice and which when in deadlock -----
            if obj.isDeadlock_
                % number of vertices of the region
                nVert = size(obj.safe_region_.verts, 1);
                obj.isAtVertex_ = 0;
                % check for each vertex
                for i = 1 : nVert
                    distance = norm(obj.pos_est_ - obj.safe_region_.verts(i, :)');
                    if distance <= obj.goal_tolerance_ && nVert >= 3
                        obj.isAtVertex_ = i;    % at which index
                        obj.ifAvoidRobotDeadlock_ = 0;  % enable robot deadlock avoiding
                        break;
                    end
                end
            end
            
            % ---- check if at edge and which hyperplane with corresponding object -----
            if obj.isDeadlock_ && ~obj.isAtVertex_
                obj.isAtEdge_ = 1;
                % check the detail edge info
                % loop for other robot
                for i = 1 : size(obj.safe_region_.Ab_robot, 1)
                    a = obj.safe_region_.Ab_robot(i, 1:obj.dim_)';
                    b = obj.safe_region_.Ab_robot(i, obj.dim_+1);
                    dist = a'*obj.pos_est_ - b;
                    if abs(dist) <= obj.goal_tolerance_
                        obj.infoAtEdge_.idx_robot = i;
                        obj.ifAvoidRobotDeadlock_ = 1;  % enable robot deadlock avoiding
                        obj.numStepRotationGoal_ = 1;   % set the required number
                    end
                end
                % loop for obs
                for j = 1 : size(obj.safe_region_.Ab_obs, 1)
                    a = obj.safe_region_.Ab_obs(j, 1:obj.dim_)';
                    b = obj.safe_region_.Ab_obs(j, obj.dim_+1);
                    dist = a'*obj.pos_est_ - b;
                    if abs(dist) <= obj.goal_tolerance_
                        obj.infoAtEdge_.idx_obs = j;
                        obj.ifAvoidObsDeadlock_ = 1;    % enable obs deadlock avoiding
                    end
                end
            end
            
            % ----- check if keeping rotational goal -----
            if obj.kStepRotationGoal_ >= obj.numStepRotationGoal_
                obj.ifAvoidRobotDeadlock_ = 0;
                obj.kStepRotationGoal_ = 0;
                obj.numStepRotationGoal_ = 1;
            end
            
        end
        
        
        %% ========== Choose current goal ==========
        function chooseCurrentGoal(obj)
            
            if obj.isArrived_ == 1                  % arrived
                obj.goal_current_ = obj.goal_final_;
            elseif obj.ifReachCurrentGoal_ == 1     % reaching current goal
                obj.goal_current_ = obj.goal_final_; 
            elseif obj.isAtVertex_ ~= 0             % not arrived, but at some vertice
                obj.rotateFinalGoal();
            elseif obj.isAtEdge_ == 1               % not arrived, but at some edge
                if obj.infoAtEdge_.idx_robot > 0    % not arrived, at edge with other robot
                    obj.rotateFinalGoal();
                end
                if obj.infoAtEdge_.idx_obs > 0      % not arrived, at edge with obs
                    % fetch info of the obs
                    obs_idx = obj.infoAtEdge_.idx_obs;
                    obs_pos = obj.obs_info_local_.pos(:,obs_idx);
                    obs_size = obj.obs_info_local_.size(:,obs_idx);
                    obs_yaw = obj.obs_info_local_.yaw(:,obs_idx);
                    obs_verts = box2PolyVertsCons(obj.dim_, obs_pos, ...
                        obs_size+2*obj.radius_, obs_yaw);   % 2*p, obs expanded with robot radius
                    % find the closest vert
                    dist_verts = vecnorm(obs_verts - obj.pos_est_);
                    [~, idx_vert] = min(dist_verts);
                    % choose as current goal
                    obj.goal_current_ = obs_verts(:, idx_vert);
                end
                if obj.infoAtEdge_.idx_robot == 0 && obj.infoAtEdge_.idx_obs == 0   % at bound edge
                    % ???
                    obj.goal_current_ = obj.goal_final_;
%                     obj.rotateFinalGoal();
                end
            elseif obj.ifAvoidObsDeadlock_ == 1     % not arrived, not deadlock, but in avoiding edge obs
                obj.goal_current_ = obj.goal_current_;
                % disable robot deadlock avoiding
%                 obj.kStepRotationGoal_ = 0;
%                 obj.ifAvoidRobotDeadlock_ = 0;
            elseif obj.ifAvoidRobotDeadlock_ == 1   % not arrived, not deadlock, but in avoiding robot vertice/edge
                obj.goal_current_ = obj.goal_current_;
                obj.kStepRotationGoal_ = obj.kStepRotationGoal_ + 1;
            else                                    % not arrived, not deadlock, not avoiding obs/robot deadlock
                obj.goal_current_ = obj.goal_final_;
            end    
            
        end        
        
        
        %% ========== Rotate current goal position temporally ==========
        function rotateFinalGoal(obj)
            
            ang_deg = rad2deg(-0.5*pi);
            R_z = rotz(ang_deg);
            R = R_z(1:obj.dim_, 1:obj.dim_);
            obj.goal_current_ = R*(obj.goal_final_ - obj.pos_est_) ...
                + obj.pos_est_;
            
        end
        
        
        %% ========== Change current goal position to the neighbor vertex temporally ==========
        function navigateToNeighborVertex2D(obj)
           
            % in this case, the robot is in deadlock and get stuck at some
            % vertex
            
            nVert = size(obj.safe_region_.verts, 1);
            if obj.isAtVertex_ > 1      % not in the first vertex
                obj.goal_current_ = obj.safe_region_.verts(obj.isAtVertex_ - 1, :);
            else
                obj.goal_current_ = obj.safe_region_.verts(nVert, :);
            end
            
        end
        
            
    end
    
    
end
