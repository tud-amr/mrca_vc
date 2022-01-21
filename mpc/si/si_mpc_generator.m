% Script for generating FORCES PRO NLP solvers 
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

%% Dynamics, i.e. equlity constraints
% RK integrator is used to discretize the continuous-time dynamics
model.eq = @(z, p) single_integrator_discrete_dynamics(z, p);
% Indices on LHS of dynamical constraint - for efficiency reasons, make
% sure the matrix E has structure [0 I] where I is the identity matrix
model.E = [zeros(model.neq, model.nin + model.nslack), eye(model.neq)];

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
model.lb = zeros(1, model.nvar);
model.ub = zeros(1, model.nvar);
% control input bound
model.lb(index.z.inputs) = -inf * ones(1, pr.dim);
model.ub(index.z.inputs) = +inf * ones(1, pr.dim);
% slacks should be always larger than zero
% model.lb(index.z.slack) = [   0];
% model.ub(index.z.slack) = [+inf];
% position is constrained using safe region
model.lb(index.z.pos) = -inf * ones(1, pr.dim);
model.ub(index.z.pos) = +inf * ones(1, pr.dim);
% general nonlinear inequalities hl <= h(x) <=hu
model.ineq = @(z, p) si_mpc_nonlinIneq(z, p, model.nHyperplanes);
% upper/lower bound for inequalities
model.hl = [-inf,     zeros(1, model.nHyperplanes)]';
model.hu = [0,    +inf*ones(1, model.nHyperplanes)]';

%% Objective function
model.objective  = @(z, p) si_objective(z, p);

%% Initial and final conditions
% here we only need to spercify on which variables initial conditions are imposed
model.xinitidx = index.z.pos;

%% Define solver options
solver_name = strcat('si_', num2str(pr.dim), 'D_', ...
    num2str(model.nHyperplanes), '_', ...
    num2str(model.N), '_', num2str(1000*model.dt));
codeoptions = getOptions(solver_name);
% codeoptions.platform    = 'Generic';% target platform
codeoptions.maxit       = 500;      % maximum number of iterations
codeoptions.printlevel  = 0;        % use printlevel = 2 to print progress
                                    % (but not for timings)
codeoptions.optlevel    = 3;        % 0: no optimization, 
                                    % 1: optimize for size, 
                                    % 2: optimize for speed, 
                                    % 3: optimize for size & speed
codeoptions.overwrite   = 1;
codeoptions.cleanup     = 1;
codeoptions.timing      = 1;
codeoptions.parallel    = 1;        % run prediction on multiple cores 
                                    % (better for longer horizons)
codeoptions.threadSafeStorage   = true; % the generated solver can be run
                                        % in parallel on different threads
codeoptions.BuildSimulinkBlock  = 0;% skipping builing of simulink S-function
% codeoptions.nlp.linear_solver   = 'symm_indefinite_fast'; 
                                    % linear system solver, better for long horizons
% codeoptions.nlp.checkFunctions  = 0;% not check the output of the function evaluations
codeoptions.noVariableElimination = 1;
codeoptions.nlp.TolStat = 1E-3;     % infinity norm tolerance on stationarity
codeoptions.nlp.TolEq   = 1E-3;     % infinity norm of residual for equalities
codeoptions.nlp.TolIneq = 1E-3;     % infinity norm of residual for inequalities
codeoptions.nlp.TolComp = 1E-3;     % tolerance on complementarity conditions
% define integrator options
% codeoptions.nlp.integrator.type = 'ERK2'; % can also be 'ForwardEuler', 
%                                           % 'ERK2', 'ERK3', 'ERK4', 
%                                           % 'BackwardEuler', or 'IRK2'
% codeoptions.nlp.integrator.Ts   = model.dt;
% codeoptions.nlp.integrator.nodes= 4;
% change this to your server or leave uncommented for using the standard
% embotech server at https://www.embotech.com/codegen
% codeoptions.server = 'http://yourforcesserver.com:8114/v1.5'; 

%% Generate FORCES PRO solver
fprintf('[%s] Generating new FORCES solver...\n',datestr(now,'HH:MM:SS'));
FORCES_NLP(model, codeoptions);
fprintf('[%s] FORCES solver generated OK \n',datestr(now,'HH:MM:SS'));

%% Storing the solver
% create a folder
folder_name = ['./solver/si_', num2str(pr.dim), 'D', '/Forces_', ...
    num2str(model.nHyperplanes), '_', ...
    num2str(model.N), '_', num2str(1000*model.dt)];
mkdir(folder_name);
% delete it and create again to remove all files in the folder
rmdir(folder_name, 's');
mkdir(folder_name);
% move the generated solver to the folder
movefile(solver_name, folder_name);         % move the folder
movefile([solver_name, '*'], folder_name);  % move the files
movefile('*forces', folder_name);
% include path
setPath;
