function [projection, exitflag] = project_to_poly_line(point, A, b, Aeq, beq)
% PROJECT_TO_POLY_LINE - Find the projection of a point to a polygon which is
%                        presented by a set of linear inequality and
%                        equality constraints 
%
% projection = project_to_poly_line(point, A, b, Aeq, beq)
%
% Input arguments:
% point - n x 1 vector (given point, n dimensional)
% A - m x n matrix, where m >= n (m constraints, n variables)
% b - m x 1 vector (m constraints)
% Aeq - p x n vector
% beq - p x 1
% 
% Output arguments:
% projection - n x 1 vector (projected point, n dimensional)
% exitflag - exitflag of the quadprog solver
%
% NOTE:
%   - The polygon is repsented by a set of linear inequality constraints
%     A*x <= b
%     Aeq*x = beq
%
% METHOD: 
%   - point = arg min (\norm(projection - point)), s.t. A*projection <= b,
%
% EXAMPLE:
% p = [1; 1];
% A = [-1, 0;
%       0, -1; 
%       1, 1];
% b = [0; 0; 1];
% Aeq = [2, -1];
% beq = 0
% pro = project_to_poly_line(p, A, b, Aeq, beq)


%% construct and solve a quadratic programming problem
dim = length(point);            % the dimension
H = eye(dim);
f = -point;

options = optimset('Display','off', 'MaxIter', 10);
[projection, ~, exitflag] = quadprog(H, f, A, b, Aeq, beq, [], [], [], options);


end
