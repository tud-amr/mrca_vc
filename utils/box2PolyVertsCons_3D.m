% transform a box to polytope verts and linear constraints
% 
% inputs:
%   - pos: box center position, x and y, [3x1]
%   - size: box size, length and width, [3x1]
%   - orientation_xy: box orientation, [1]
% 
% outputs: 
%   - poly_vert: vertice represented the box, 3*m
%   - poly_Ab: linear constraints represented half planes of the box
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [poly_vert, poly_Ab] = box2PolyVertsCons_3D(pos, size, orientation_xy)
    
%     poly_vert = zeros(3, 8);            % 8 vert
%     poly_Ab   = zeros(4, 6);            % 6 faces
    
    %% vertices
    % before rotation and translation
    x_min = - 0.5*size(1);
    x_max = + 0.5*size(1);
    y_min = - 0.5*size(2);
    y_max = + 0.5*size(2);
    z_min = - 0.5*size(3);
    z_max = + 0.5*size(3);
    p1 = [x_min; y_min; z_min];
    p2 = [x_max; y_min; z_min];
    p3 = [x_max; y_max; z_min];
    p4 = [x_min; y_max; z_min];
    p5 = [x_min; y_min; z_max];
    p6 = [x_max; y_min; z_max];
    p7 = [x_max; y_max; z_max];
    p8 = [x_min; y_max; z_max];
    % only rotate in yaw (xy plane), then translation
    rot_mtx = rotz(rad2deg(orientation_xy));
    p1_r = rot_mtx * p1 + pos;
    p2_r = rot_mtx * p2 + pos;
    p3_r = rot_mtx * p3 + pos;
    p4_r = rot_mtx * p4 + pos;
    p5_r = rot_mtx * p5 + pos;
    p6_r = rot_mtx * p6 + pos;
    p7_r = rot_mtx * p7 + pos;
    p8_r = rot_mtx * p8 + pos;
    % polytope verts
    poly_vert = [p1_r, p2_r, p3_r, p4_r, p5_r, p6_r, p7_r, p8_r];

    %% half plane constraints
    [A, b] = vert2con(poly_vert');
    poly_Ab(1:3, :) = A';
    poly_Ab(4, :) = b';
    
    %% normalization the A matrix
    for i = 1 : 6
        a = poly_Ab(1:3, i);
        poly_Ab(1:3, i) = poly_Ab(1:3, i) / norm(a);
        poly_Ab(4, i) = poly_Ab(4, i) / norm(a);
    end

end
