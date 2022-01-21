% transform a box to polytope verts and linear constraints
% 
% inputs:
%   - pos: box center position, x and y, [2x1]
%   - size: box size, length and width, [2x1]
%   - orientation_xy: box orientation, [1]
% 
% outputs: 
%   - poly_vert: vertice represented the box, 2*m
%   - poly_Ab: linear constraints represented half planes of the box
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [poly_vert, poly_Ab] = box2PolyVertsCons_2D(pos, size, orientation_xy)
    
    poly_vert = zeros(2, 4);
    poly_Ab   = zeros(3, 4);
    
    %% vertices
    % before rotation and translation
    x_min = - 0.5*size(1);
    x_max = + 0.5*size(1);
    y_min = - 0.5*size(2);
    y_max = + 0.5*size(2);
    p1 = [x_min; y_min];
    p2 = [x_max; y_min];
    p3 = [x_max; y_max];
    p4 = [x_min; y_max];
    % only rotate in xy plane, then translation
    rot_mtx = rotz(rad2deg(orientation_xy));
    p1_r = rot_mtx(1:2, 1:2) * p1 + pos;
    p2_r = rot_mtx(1:2, 1:2) * p2 + pos;
    p3_r = rot_mtx(1:2, 1:2) * p3 + pos;
    p4_r = rot_mtx(1:2, 1:2) * p4 + pos;
    % polytope verts
    poly_vert = [p1_r, p2_r, p3_r, p4_r];

    %% half plane constraints
    [A, b] = vert2con(poly_vert');
    poly_Ab(1:2, :) = A';
    poly_Ab(3, :) = b';
    
    %% normalization the A matrix
    for i = 1 : 4
        a = poly_Ab(1:2, i);
        poly_Ab(1:2, i) = poly_Ab(1:2, i) / norm(a);
        poly_Ab(3, i) = poly_Ab(3, i) / norm(a);
    end

end

%% test
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -2 2]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% hold on;
% box_center = [0.5; 1];
% box_size = [2; 1];
% box_yaw = deg2rad(0);
% hp = plot_box_2D(ax, box_center, box_size, box_yaw, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'FaceAlpha', 0.2, ...
%     'EdgeColor', 'k', ...
%     'EdgeAlpha', 0.4, ...
%     'LineWidth', 1.0, ...
%     'LineStyle', '-', ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5);