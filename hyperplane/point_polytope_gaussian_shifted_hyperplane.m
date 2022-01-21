% compute a normalized max-margin shifted separating hyperplane between 
% a point and a polytope with position uncertainty covariance
% 
% inputs:
%   - p: point 1, [dxk], d is the dimension
%   - cov: uncertainty covariance, [dxd]
%   - vert: set of vertice to represent the polytope, [dxp]
%   - delta: collision probability threshold
% 
% outputs: 
%   - a, b: hyperplane parameters s.t. |a|=1 and a'*p < b, a'*vert(:,i) > b
%   a: [dx1], b: [1]
% 
% note: using the SVM maximum margin classifier method
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [a, b] = point_polytope_gaussian_shifted_hyperplane(p, cov, vert, delta)
    
    d = length(p);                          % dimension

    % coordinate transformation
    W = (sqrt(cov))^(-1);               % W
%     W = zeros(d, d);
%     for i = 1 : d
%         W(i, i) = 1 / (sqrt(cov(i, i)));    % if cov is diagonal
%     end
    
    % perform transformation
    p_t = W*p;
    vert_t = W*vert;
    
    % compute obstacle shadow
    eps = sqrt(delta);
    r_square = icdf('Chisquare', 1-eps, d);
    r = sqrt(r_square);
    
    vert_t_expand = expandPolytope(vert_t', 2*r);
    vert_t_expand = vert_t_expand';
    
    % compute the max-margin hyperplane in transformed space
    [a_t, b_t] = point_polytope_shifted_hyperplane(p_t, vert_t_expand);
    
    % inverse transformation
    a = W' * a_t;
    b = b_t;   
    
    % normalization
    a_norm = norm(a);
    a = a ./ a_norm;
    b = b ./ a_norm;

end


%% test script
%% 2D
% p1 = [-3; 1];
% p2 = [1; 0];
% p = p1; %[p1, p2];
% delta = 0.03;
% box_center = [0.5; 1];
% box_cov = [0.1^2, 0.4*0.1*0.2; 0.4*0.1*0.2, 0.2^2];
% box_size = [3; 1.6];
% box_yaw = deg2rad(0);
% box_vert = box2PolyVertsCons_2D(box_center, box_size, box_yaw);
% tic
% [a, b] = point_polytope_shifted_hyperplane(p, box_vert);
% [a_g, b_g] = point_polytope_gaussian_shifted_hyperplane(p, box_cov, box_vert, delta);
% toc
% hfig = figure;
% box on;
% grid on;
% axis([-4 4 -4 4]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% hold on;
% plot(p(1, :), p(2, :), 'o');
% hb = plot_poly_vert_2D(ax, box_vert, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'FaceAlpha', 0.6, ...
%     'EdgeColor', [0.4 0.4 0.4], ...
%     'EdgeAlpha', 0.8, ...
%     'LineWidth', 1.0, ...
%     'LineStyle', '-.', ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5);
% dim = [-4 4; -4 4];
% hl = plot_line_2D(ax, a, b, dim, ...
%     'Color', [0.4 0.4 0.4], ...
%     'LineStyle', '--', ...
%     'LineWidth', 1.5);
% hg = plot_line_2D(ax, a_g, b_g, dim, ...
%     'Color', 'r', ...
%     'LineStyle', '-', ...
%     'LineWidth', 1.5);
%% 3D
% p1 = [-3; 1; 1.6];
% p2 = [1; 0; 1.6];
% p = p1; %[p1, p2];
% delta = 0.03;
% box_center = [0.5; 1; 1.6];
% box_cov = diag([0.1, 0.12, 0.1].^2);
% box_size = [3; 1.6; 3.2];
% box_yaw = deg2rad(30);
% box_vert = box2PolyVertsCons_3D(box_center, box_size, box_yaw);
% tic
% [a, b] = point_polytope_shifted_hyperplane(p, box_vert);
% [a_g, b_g] = point_polytope_gaussian_shifted_hyperplane(p, box_cov, box_vert, delta);
% toc
% hfig = figure;
% box on;
% grid on;
% axis([-4 4 -4 4 0 4]);
% ax = hfig.CurrentAxes;
% view(ax, 3);
% daspect(ax, [1 1 1]);
% hold on;
% plot3(p(1, :), p(2, :), p(3, :),'o');
% hb = plot_poly_vert_3D(ax, box_vert, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'FaceAlpha', 0.6, ...
%     'EdgeColor', [0.4 0.4 0.4], ...
%     'EdgeAlpha', 0.8, ...
%     'LineWidth', 1.0, ...
%     'LineStyle', '-.', ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5);
% dim = [-4 4; -4 4; 0 4];
% hl = plot_plane_3D(ax, a, b, dim, ...
%     'FaceColor', 'b', ...
%     'EdgeColor', 'b', ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5, ...
%     'FaceAlpha', 0.6);
% hg = plot_plane_3D(ax, a_g, b_g, dim, ...
%     'FaceColor', 'r', ...
%     'EdgeColor', 'r', ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5, ...
%     'FaceAlpha', 0.6);