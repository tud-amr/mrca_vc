% plot a jackal in 2D and return the plot handle
% 
% inputs:
%   - ax: figure handle
%   - pos: quadrotor position center, [2x1], m
%   - yaw: orientation of the robot, rad
%   - radius: robot radius
%   - color: robot color
%   - varargin: patch properties
% 
% outputs: 
%   - h: plot handle
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
% 

function h = plot_jackal_2D(ax, pos, yaw, radius, color, varargin)

    % determine outbound points
    r = 0.6*radius;
    p1 = [-r; -1.2*r];
    p2 = [-r; 0.8*r];
    p3 = [0; 1.6*r];
    p4 = [r; 0.8*r];
    p5 = [r; -1.2*r];
    
    % transformation
    rot_mtx = rotz(rad2deg(yaw));
    R = rot_mtx(1:2, 1:2);
    p1 = R*p1 + pos;
    p2 = R*p2 + pos;
    p3 = R*p3 + pos;
    p4 = R*p4 + pos;
    p5 = R*p5 + pos;
    p = [p1, p2, p3, p4, p5];
    
    % patch
    h_out = patch(ax, ...
        p(1, :), p(2, :), ...
        color, 'LineWidth', 1.5, varargin{:});
    h_pos = plot(ax, pos(1), pos(2), 'o', ...
        'MarkerSize', 4, 'MarkerEdgeColor', color, ...
        'MarkerFaceColor', color);
    h_circle = plot_ellipse_2D(ax, pos, [radius; radius], yaw, varargin{:});
    h = [h_out; h_pos; h_circle];
end


%% test script
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -3 3]);
% ax = hfig.CurrentAxes;
% axis equal
% pos = [1; 0.5];
% yaw = deg2rad(30);
% color = 'b';
% radius = 0.2;
% h = plot_jackal_2D(ax, pos, yaw, radius, color, ...
%     'FaceColor', color, 'FaceAlpha', 0.1, ...
%     'EdgeColor', color, 'EdgeAlpha', 0.8);