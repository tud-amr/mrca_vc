classdef CBoxObs < handle
    % Common base class for box obstacles
    
    properties
        
        %% id, configuration
        dim_            =   0;              % dimension
        id_             =   0;              % index
        
        %% state
        pos_real_       =   [];             % centroid position
        pos_est_        =   [];             % estimated
        pos_est_cov_    =   [];             % position uncertainty
        
        if_sim_noise_   =   0;              % if simulating observation noise
        
        %% size
        size_           =   [];             % length in each dim
        yaw_            =   [];             % yaw orientation in xy plane
        
        %% polyhedraon
        poly_A_         =   [];             % represented as linear constraints, m*dim
        poly_b_         =   [];             % m*1
        poly_vert_      =   [];             % vertices, m*dim
        
    end
    
    
    methods
        
        %%  =======================================================================
        function obj = CBoxObs(pr, obsID)
            % Constructor
            
            obj.dim_    =   pr.dim;
            obj.id_     =   obsID;
            obj.if_sim_noise_ = pr.ifSimBoxObsNoise;
            
            % vector and matrix initialization
            obj.pos_real_   =   zeros(obj.dim_, 1);
            obj.pos_est_    =   zeros(obj.dim_, 1);
            obj.pos_est_cov_=   0.1^2 * eye(obj.dim_);
            
            obj.size_       =   zeros(obj.dim_, 1);
            obj.yaw_        =   0;
            
        end
        
        
        %%  =======================================================================
        function initializeState(obj, pos, pos_cov, size, yaw)
            % Initilize obs state
            
            obj.pos_real_    =   pos;
            obj.pos_est_cov_ =   pos_cov;
            
            obj.size_        =   size;
            obj.yaw_         =   yaw;
            
        end
        
        
        %%  =======================================================================
        function getPolyVertsCons(obj)
            % Box obstalce represented by polytope verts, or linear
            % constraints
            
            switch obj.dim_
                case 2
                    [poly_vert, poly_Ab] = box2PolyVertsCons_2D(...
                        obj.pos_real_, obj.size_, obj.yaw_);
                otherwise
            end
            
            obj.poly_vert_ = poly_vert';        % m*dim
            obj.poly_A_ = poly_Ab(1:2,:)';      % m*dim
            obj.poly_b_ = poly_Ab(3,:)';        % m*1
            
        end
        
        
        %%  =======================================================================
        function getEstimatedObsState(obj)
            % Obtain estimated pos
            
            obj.pos_est_ = obj.pos_real_;
            
            if obj.if_sim_noise_ == 1
                dpos = zeros(obj.dim_, 1);
                for i = 1 : obj.dim_
                    dpos(i) = random('Normal', 0, ...
                        sqrt(obj.pos_est_cov_(i,i)));
                end
                obj.pos_est_ = obj.pos_est_ + dpos;
            end
            
        end
        
        
        %%  =======================================================================
        
           
        
        %%  =======================================================================

        
        
        %%  =======================================================================
        
        
        
        %%  =======================================================================

        
        
        %%  =======================================================================

        
        
        %%  =======================================================================

        
        
    end
    
    
    
end