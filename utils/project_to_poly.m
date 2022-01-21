function [projection, exitflag] = project_to_poly(point, A, b)
% PROJECT_TO_POLY - Find the projection of a point to a polygon which is
%                   presented by a set of linear inequality constraints
%
% projection = project_to_poly(point, A, b)
%
% Input arguments:
% point - n x 1 vector (given point, n dimensional)
% A - m x n matrix, where m >= n (m constraints, n variables)
% b - m x 1 vector (m constraints)
% 
% Output arguments:
% projection - n x 1 vector (projected point, n dimensional)
% exitflag - exitflag of the quadprog solver
%
% NOTE:
%   - The polygon is repsented by a set of linear inequality constraints
%     A*x <= b
%
% METHOD: 
%   - point = arg min (\norm(projection - point)), s.t. A*projection <= b
%
% EXAMPLE:
% p = [1; 1];
% A = [-1, 0;
%       0, -1; 
%       1, 1];
% b = [0; 0; 1];
% pro = project_to_poly(p, A, b)


%% construct and solve a quadratic programming problem
dim = length(point);            % the dimension
H = eye(dim);
f = -point;

options = optimset('Display','off', 'MaxIter', 30);
[projection, ~, exitflag] = quadprog(H, f, A, b, [], [], [], [], [], options);
if isempty(projection)
    projection = point;
end

end
