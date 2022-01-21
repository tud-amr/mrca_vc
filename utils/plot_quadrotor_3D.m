% plot a quadrotor in 3D and return the plot handle
% 
% inputs:
%   - ax: figure handle
%   - pos: quadrotor position center, [3x1]
%   - R: quadrotor orientation rotation matrix, [3x3]
%   - radius: quadrotor radius
%   - color: quadrotor color
%   - varargin: patch properties
% 
% outputs: 
%   - h: plot handle
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
% 

function h = plot_quadrotor_3D(ax, pos, R, radius, color, varargin)
    
    % quad size
    diameter = 2.0 * radius;
    beam_dia = 0.24 * diameter;
    prop_dia = 0.15 * diameter;
    line_width = 3.0 * diameter;
    
    % center of the four propellers
    prop_center = zeros(4, 3);
    prop_center(1,:) = pos + R * beam_dia * [-1, 1, 0]';
    prop_center(2,:) = pos + R * beam_dia * [ 1, 1, 0]';
    prop_center(3,:) = pos + R * beam_dia * [ 1,-1, 0]';
    prop_center(4,:) = pos + R * beam_dia * [-1,-1, 0]';
    
    % beam
    h_beam(1) = line(ax, ...
                    [prop_center(1,1), prop_center(3,1)], ...
                    [prop_center(1,2), prop_center(3,2)], ...
                    [prop_center(1,3), prop_center(3,3)], ...
                    'Color', color, ...
                    'LineWidth', line_width);
    h_beam(2) = line(ax, ...
                    [prop_center(2,1), prop_center(4,1)], ...
                    [prop_center(2,2), prop_center(4,2)], ...
                    [prop_center(2,3), prop_center(4,3)], ...
                    'Color', color, ...
                    'LineWidth', line_width);
    
    % propeller
    for i = 1 : 4
        % points
        rotor_points = pointsCircle3D(prop_center(i,:), R(:,3).', prop_dia);
        h_propeller(i) = patch(ax, ...
            rotor_points(1,:), rotor_points(2,:), rotor_points(3,:), ...
           	color, 'LineWidth', 1.5, varargin{:});
    end
    
    % head
    head = pos + beam_dia*R(:,1);
    h_head = line(ax, [pos(1), head(1)], ...
                    [pos(2), head(2)], ...
                    [pos(3), head(3)], ...
                    'Color', color, 'LineWidth', 2.0);
                
    h = {h_beam; h_propeller; h_head};
end

function points = pointsCircle3D(center, normal, radius)
    % Create points for 3D circle
    theta = 0 : 0.1 : 2*pi+0.1;
    v = null(normal);               % Null space (normal*v = 0);
    points = repmat(center', 1, ...
        size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
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
% pos = [1; 0.5; 1];
% euler = deg2rad([8; 12; 30]);
% R = rotXYZ(euler);
% color = 'b';
% radius = 0.4;
% h = plot_quadrotor_3D(ax, pos, R, radius, color, ...
%     'FaceColor', color, 'FaceAlpha', 0.1, ...
%     'EdgeColor', color, 'EdgeAlpha', 0.8);