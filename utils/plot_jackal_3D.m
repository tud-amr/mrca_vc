% plot a jackal in 3D and return the plot handle
% 
% inputs:
%   - ax: figure handle
%   - pos: quadrotor position center, [3x1], m
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

function h = plot_jackal_3D(ax, pos, yaw, height, radius, color, varargin)

    % determine outbound points
    r = 0.75*radius;
    p1 = [-r; -r; 0];
    p2 = [-r; r; 0];
    p3 = [0; 2*r; 0];
    p4 = [r; r; 0];
    p5 = [r; -r; 0];
    
    % transformation
    rot_mtx = rotz(rad2deg(yaw));
    R = rot_mtx(1:3, 1:3);
    p1 = R*p1 + pos;
    p2 = R*p2 + pos;
    p3 = R*p3 + pos;
    p4 = R*p4 + pos;
    p5 = R*p5 + pos;
    p = [p1, p2, p3, p4, p5];
    
    % patch
    h_out = patch(ax, ...
        p(1, :), p(2, :), p(3, :), ...
        color, 'LineWidth', 1.5, varargin{:});
    h_pos = plot3(ax, pos(1), pos(2), pos(3), 'o', ...
        'MarkerSize', 8, 'MarkerEdgeColor', color, ...
        'MarkerFaceColor', color);
    
    % box
    box_pos = [pos(1); pos(2); height/2];
    box_size = [2*r; 2*r; height];
    box_yaw = yaw;
    h_box = plot_box_3D(ax, box_pos, box_size, box_yaw, varargin{:});
    
    h = [h_out; h_pos; h_box];
end


%% test script
% hfig = figure;
% hold on;
% box on;
% grid on;
% axis([-3 3 -3 3 0 2.5]);
% ax = hfig.CurrentAxes;
% view(ax, 3);
% pos = [1; 0.5; 0];
% height = 1.0;
% yaw = deg2rad(30);
% color = 'b';
% radius = 0.2;
% h = plot_jackal_3D(ax, pos, yaw, height, radius, color, ...
%     'FaceColor', color, 'FaceAlpha', 0.1, ...
%     'EdgeColor', color, 'EdgeAlpha', 0.8);