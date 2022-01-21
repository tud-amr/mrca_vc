% Script of problem setup, mpc for single integrator dynamics
% 
% Define problem dimensions and indexing here
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

%% Problem parameters
global model                        % the model setup is accessed globally
model.nHyperplanes  =   12  ;       % maximal 12 hyperplanes for safe region constraints
model.nParaEachPlane=   pr.dim + 1; % parameters for each plan
model.nParamSafeRegion = model.nHyperplanes*model.nParaEachPlane;      
                                    % number of parameters for safe region hyperplanes
model.N             =   pr.N;       % horizon length
model.dt            =   pr.dtSim;   % time step
model.nvar          =   2*pr.dim;   % number of stage variables (z) [vel, pos]
model.neq           =   pr.dim;     % number of equality constraints (x)
model.nh            =   model.nHyperplanes + 1; 
                                    % number of inequality constraints
                                    % velocity speed limit is considered
model.nin           =   pr.dim;     % number of control inputs (u) [vel]
model.nslack        =   0   ;       % number of slacks (s)
model.npar          =   4 + 2*pr.dim + model.nParamSafeRegion;
                                    % number of runtime parameters on each
                                    % stage

%% Indexing, not changable when running
global index                        % the index is used globally
% in stage vector, each stage
index.z.all         =   1:model.nvar;
index.z.inputs      =   1:pr.dim;   % control input, [vel]
index.z.slack       =   pr.dim;     % slack, not used
index.z.pos         =   index.z.slack(end)+1:index.z.slack(end)+pr.dim;  % position, [pos]
% in state vector, each stage
index.x.all         =   1:model.neq;
index.x.pos         =   1:pr.dim;
% in parameter vector, problem, each stage
index.p.all         =   1:model.npar;
index.p.startPos    =   1:pr.dim;   % [pos0]
index.p.wayPoint    =   pr.dim+1:2*pr.dim;  % [posg]
index.p.dt          =   2*pr.dim+1;         % dt
index.p.speedLimit  =   2*pr.dim+2;         % v_max
index.p.weights     =   2*pr.dim+3:2*pr.dim+4;  % [w_wp_pos, w_input_v]
% in parameter vector, safe region
idxBegin = index.p.weights(end) + 1;
index.p.safeRegion = reshape(idxBegin : ((idxBegin-1)+(pr.dim+1)*model.nHyperplanes),...
        [(pr.dim+1), model.nHyperplanes]);
index.p.hyperplane.a = 1:pr.dim;
index.p.hyperplane.b = pr.dim+1;
