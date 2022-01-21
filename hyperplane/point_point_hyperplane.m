% compute a normalized hyperplane to separate two points
% 
% inputs:
%   - p1: point 1, [dx1], d is the dimension
%   - p2: point 2, [dx1]
% 
% outputs: 
%   - a, b: hyperplane parameters s.t. |a|=1 and a'*p1 < b, a'*p2 > b
%   a: [dx1], b: [1]
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [a, b] = point_point_hyperplane(p1, p2)
    
    p12 = p2 - p1;
    p12_norm = norm(p12);
    
    assert(p12_norm > 10E-16, 'Two points coincide!');

    a = p12 / p12_norm;
    b = 0.5*transpose(a) * (p1+p2);

end

%% test script
%% 2D
% hfig = figure;
% box on;
% grid on;
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% hold on;
% dim = [-3 3; -2 2];
% p1 = [-2; -1];
% p2 = [2; 1];
% plot([p1(1), p2(1)], [p1(2), p2(2)], '-o')
% [a, b] = point_point_hyperplane(p1, p2);
% hp = plot_line_2D(ax, a, b, dim, ...
%     'Color', [0.4 0.4 0.4], ...
%     'LineStyle', '--', ...
%     'LineWidth', 1.5);
%% 3D
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -3 3 -2 2]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% rotate3d(ax);
% view(ax, 3);
% hold on;
% p1 = [-2; -1; -1];
% p2 = [3; 1; 2];
% plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], '-o')
% [a, b] = point_point_hyperplane(p1, p2);
% dim = [-3 3; -3 3; -100 100];
% hp = plot_plane_3D(ax, a, b, dim, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'EdgeColor', [0.4 0.4 0.4], ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5, ...
%     'FaceAlpha', 0.6);
