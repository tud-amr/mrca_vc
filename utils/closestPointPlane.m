% find the closest point on a hyperplane to a given point, and the distance
% 
% inputs:
%   - a: hyperplane para, [nx1]
%   - b: hyperplane para, [1], s.t. a'*x = b
%   - p0: given point, [nx1]
% 
% outputs: 
%   - p: the closest point
%   - d: the distance
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [p, d] = closestPointPlane(a, b, p0)

    a_n = a / norm(a);
    b_n = b / norm(a);
    
    if a_n'*p0 - b <= 0     % reverse normal
        a_n = -a_n;
        b_n = -b_n;
    end
    
    d = abs(a_n'*p0 - b_n);
    p = p0 - a_n*d;

end


%% test script
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -3 3 -2 2]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% rotate3d(ax);
% view(ax, 3);
% hold on;
% a = [1; 1; 1];
% b = 2;
% dim = [-3 3; -3 3; -100 100];
% hp = plot_plane_3D(ax, a, b, dim, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'EdgeColor', [0.4 0.4 0.4], ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5, ...
%     'FaceAlpha', 0.6);
% p0 = [-2; 1; 1];
% [p, d] = closestPointPlane(a, b, p0);
% plot3(ax, [p0(1) p(1)], [p0(2) p(2)], [p0(3) p(3)], '-r');
