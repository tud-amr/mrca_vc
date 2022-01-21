classdef CSystem < handle
    % Class for a multi-robot system
    
    properties
        
        %% setup
        dim_                                % dimension
        dt_                                 % delta t for simulation and control
        ws_                                 % workspace
        
        %% timer
        time_global_            =   0;
        time_step_global_       =   0;
        
        %% object
        nRobot_                 =   0;
        nBoxObs_                =   0;
        MultiRobot_             =   {};
        MultiBoxObs_            =   {};
        
        %% system current real state
        multi_robot_goal_real_  =   [];     % goal
        multi_robot_pos_real_   =   [];     % pos
        multi_obs_pos_real_     =   [];
        multi_obs_size_real_    =   [];
        multi_obs_yaw_real_     =   [];   
        multi_obs_vert_real_    =   [];
        
        %% system current est state
        multi_robot_pos_est_    =   [];     % pos
        multi_robot_pos_est_cov_=   [];     % pos est cov
        multi_obs_pos_est_      =   [];
        multi_obs_pos_est_cov_  =   [];
        
        %% collision info
        collision_mtx_          =   [];
        
        %% method
        method_                 =   0;      % 0 - BVC; 1 - BUAVC
        control_                =   0;      % 0 - one-step; 1 - mpc
        
    end
    
    
    methods
        
        %%  =======================================================================
        function obj = CSystem(nRobot, nBoxObs, pr)
            
            obj.dim_    =   pr.dim;
            obj.dt_     =   pr.dtSim;
            obj.ws_     =   pr.ws;
            
            obj.method_ =   pr.method;
            obj.control_=   pr.control;

            obj.time_global_        =   0;
            obj.time_step_global_   =   0;
            
            obj.nRobot_ =   nRobot;
            obj.nBoxObs_=   nBoxObs;
            
            for iRobot = 1 : nRobot
                if pr.robot_type == 0
                    obj.MultiRobot_{iRobot} = CSingleInt(pr, iRobot);
                elseif pr.robot_type == 1
                    obj.MultiRobot_{iRobot} = CDiffDrive(pr, iRobot);
                else
                    error('Robot type not defined!')
                end
            end
            
            for jBoxObs = 1 : nBoxObs
                obj.MultiBoxObs_{jBoxObs} = CBoxObs(pr, jBoxObs);
            end
            
            % vector and matrix initialization
            obj.multi_robot_pos_real_ 	=   zeros(obj.dim_, nRobot);
            obj.multi_robot_goal_real_ 	=   zeros(obj.dim_, nRobot);
            obj.multi_obs_pos_real_   	=   zeros(obj.dim_, nBoxObs);
            obj.multi_obs_size_real_  	=   zeros(obj.dim_, nBoxObs);
            obj.multi_obs_yaw_real_  	=   zeros(1, nBoxObs);
            
            obj.multi_robot_pos_est_ 	=   zeros(obj.dim_, nRobot);
            obj.multi_robot_pos_est_cov_=   zeros(obj.dim_, obj.dim_, nRobot);
            obj.multi_obs_pos_est_      =   zeros(obj.dim_, nBoxObs);
            obj.multi_obs_pos_est_cov_  =   zeros(obj.dim_, obj.dim_, nBoxObs);
            
            obj.collision_mtx_          =   zeros(nRobot, nRobot+nBoxObs);
            
        end
        
        
        %%  =======================================================================
        function getSystemState(obj)
            % Obtain sytem state, real and est
                      
            for iRobot = 1 : obj.nRobot_
                % real
                obj.multi_robot_pos_real_(:, iRobot)  = obj.MultiRobot_{iRobot}.pos_real_;
                obj.multi_robot_goal_real_(:, iRobot) = obj.MultiRobot_{iRobot}.goal_final_;
                % est
                obj.MultiRobot_{iRobot}.getEstimatedState();
                obj.multi_robot_pos_est_(:, iRobot) = obj.MultiRobot_{iRobot}.pos_est_;
                obj.multi_robot_pos_est_cov_(:,:,iRobot) = obj.MultiRobot_{iRobot}.pos_est_cov_;
            end
            
            for jBoxObs = 1 : obj.nBoxObs_
                % real
                obj.multi_obs_pos_real_(:, jBoxObs)   = obj.MultiBoxObs_{jBoxObs}.pos_real_;
                obj.multi_obs_size_real_(:, jBoxObs)  = obj.MultiBoxObs_{jBoxObs}.size_;
                obj.multi_obs_yaw_real_(:, jBoxObs)   = obj.MultiBoxObs_{jBoxObs}.yaw_;
                obj.multi_obs_vert_real_(:, :, jBoxObs) = box2PolyVertsCons(obj.dim_, ...
                    obj.multi_obs_pos_real_(:, jBoxObs), ...
                    obj.multi_obs_size_real_(:, jBoxObs), ...
                    obj.multi_obs_yaw_real_(:, jBoxObs));
                % est
                obj.MultiBoxObs_{jBoxObs}.getEstimatedObsState();
                obj.multi_obs_pos_est_(:, jBoxObs) = obj.MultiBoxObs_{jBoxObs}.pos_est_;
                obj.multi_obs_pos_est_cov_(:,:,jBoxObs) = obj.MultiBoxObs_{jBoxObs}.pos_est_cov_;
            end
            
        end
        
        
        %%  =======================================================================
        function simSystemOneStep(obj)
            % Simulate the system for one step, given current system state
            obj.time_step_global_ = obj.time_step_global_ + 1;
            
            % sim for each robot using current system state
            for iRobot = 1 : obj.nRobot_
                
                % ===== ego robot estimated state =====
                obj.MultiRobot_{iRobot}.pos_est_ = obj.multi_robot_pos_est_(:, iRobot);
                obj.MultiRobot_{iRobot}.pos_est_cov_ = obj.multi_robot_pos_est_cov_(:,:,iRobot);
                
                % ===== compute local bound =====
                obj.MultiRobot_{iRobot}.computeLocalBound();
                
                % ===== obtain local robot info =====
                obj.MultiRobot_{iRobot}.getLocalRobotInfo(...
                    obj.multi_robot_pos_est_, obj.multi_robot_pos_est_cov_);
                
                % ===== obtain local box obs info =====
                obj.MultiRobot_{iRobot}.getLocalObsInfo(...
                    obj.multi_obs_pos_est_, obj.multi_obs_pos_est_cov_, ...
                    obj.multi_obs_size_real_, obj.multi_obs_yaw_real_);
                
                % ===== deadlock checking =====
                obj.MultiRobot_{iRobot}.deadlockChecking();
                
                % ===== choose current goal for avoiding deadlock =====
                obj.MultiRobot_{iRobot}.chooseCurrentGoal();
                
                % ===== compute local safe convex region hyperplanes =====
                if obj.method_ == 0
                    obj.MultiRobot_{iRobot}.computeBVC();
                elseif obj.method_ == 1
                    obj.MultiRobot_{iRobot}.computeBUAVC();
                end
                
                % ===== compute control input =====
                if obj.control_ == 0
                    obj.MultiRobot_{iRobot}.computeControlInput();
                elseif obj.control_ == 1
                    obj.MultiRobot_{iRobot}.computeControlInputMPC();
                end
                
                % if arrived, set to 0
                if obj.MultiRobot_{iRobot}.isArrived_ == 1
                    obj.MultiRobot_{iRobot}.u_ = 0*obj.MultiRobot_{iRobot}.u_;
                end
                
                % if in collision, set to 0
                if obj.MultiRobot_{iRobot}.isCollision_ >= 1
                    obj.MultiRobot_{iRobot}.u_ = 0*obj.MultiRobot_{iRobot}.u_;
                    fprintf('Robot %d in collision!\n', iRobot);
                end
                
                % some heuristics
%                 if norm(obj.MultiRobot_{iRobot}.u_) < 0.2
%                     obj.MultiRobot_{iRobot}.u_ = 0*obj.MultiRobot_{iRobot}.u_;
%                 end
                
                % ===== simulate the robot one step =====
                obj.MultiRobot_{iRobot}.simulateOneStep();
                
            end
            
        end
        
        
        %%  =======================================================================
        function collisionChecking(obj)
            % Check if collision happens in the system
            
            obj.collision_mtx_ = collision_check(obj.nRobot_, obj.nBoxObs_, ...
                obj.multi_robot_pos_real_, obj.MultiRobot_{1}.radius_, ...
                obj.multi_obs_vert_real_);
            
            for iRobot = 1 : obj.nRobot_
                if obj.MultiRobot_{iRobot}.isCollision_ == 0
                    obj.MultiRobot_{iRobot}.isCollision_ = ...
                        min(1, sum(obj.collision_mtx_(iRobot, :)));
                end
            end
            
        end
        
        
    end
    
    
end