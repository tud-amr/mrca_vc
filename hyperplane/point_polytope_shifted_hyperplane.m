% compute a normalized max-margin shifted separating hyperplane between 
% a point and a polytope
% 
% inputs:
%   - p: point 1, [dx1], d is the dimension
%   - vert: set of vertice to represent the polytope, [dxm]
% 
% outputs: 
%   - a, b: hyperplane parameters s.t. |a|=1 and a'*p < b, a'*vert(:,i) > b
%   a: [dx1], b: [1]
% 
% note: using the SVM maximum margin classifier method
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [a, b] = point_polytope_shifted_hyperplane(p, vert)
    
    [dim1, N] = size(p);
    [dim2, M] = size(vert);
    assert(dim1 == dim2, 'Dimension error!');
    
    % maximum margin classifier method
    H = diag([ones(1, dim1), 0]);
    f = zeros(dim1+1, 1);
    A = [p', ones(N, 1); ...
         -vert', -ones(M, 1)];
    b = -ones(M+N, 1);
    options = optimset('Display', 'off');
    sol = quadprog(H, f, A, b, [], [], [], [], [], options);
    
    a = sol(1:dim1);
    b = -sol(end);
    
    % normalization
    a_norm = norm(a);
    a = a ./ a_norm;
    b = b ./ a_norm;
    
    % shifted to be tight with the polytope
    min_dis = min(a'*vert - b);
    b = b + min_dis;

end


%% test script
%% 2D
% p = [-2; 1];
% box_center = [0.5; 1];
% box_size = [2; 1];
% box_yaw = deg2rad(30);
% box_vert = box2PolyVertsCons_2D(box_center, box_size, box_yaw);
% tic
% [a, b] = point_polytope_shifted_hyperplane(p, box_vert);
% toc
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -3 3]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% hold on;
% plot(p(1), p(2), 'o');
% hb = plot_poly_vert_2D(ax, box_vert, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'FaceAlpha', 0.2, ...
%     'EdgeColor', [0.4 0.4 0.4], ...
%     'EdgeAlpha', 0.6, ...
%     'LineWidth', 1.0, ...
%     'LineStyle', '-.', ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5);
% dim = [-3 3; -2 2];
% hl = plot_line_2D(ax, a, b, dim, ...
%     'Color', [0.4 0.4 0.4], ...
%     'LineStyle', '--', ...
%     'LineWidth', 1.5);