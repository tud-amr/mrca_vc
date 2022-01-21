% plot a box represented in 2D
% 
% inputs:
%   - ax: figure handle
%   - pt: box center, [dx1]
%   - size: box size, [dx1]
%   - yaw: box yaw, [1]
%   - varargin: patch properties
% 
% outputs: 
%   - h: plot handle
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
% 

function h = plot_box_3D(ax, box_pos, box_size, box_yaw, varargin)

    % convert to verts
    [v, ~] = box2PolyVertsCons_3D(box_pos, box_size, box_yaw);

    if isempty(v)
      return
    end
    
    if ((size(v,2)<10) && (isCoplanar(v(1,:)',v(2,:)',v(3,:)',0.001)))
        X = v(1,:);
        Y = v(2,:);
        Z = v(3,:);
        h = patch(ax, X,Y,Z, 'k', varargin{:});
    else
        k = convhull(v(1,:), v(2,:), v(3,:));
        X = reshape(v(1,k'), size(k'));
        Y = reshape(v(2,k'), size(k'));
        Z = reshape(v(3,k'), size(k'));
        h = patch(ax, X,Y,Z, 'k', varargin{:});
    end
    
end

%% test script
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -2 2 0 10]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% view(ax, 3);
% hold on;
% box_center = [0.5; 1; 2.0];
% box_size = [2; 1; 4.0];
% box_yaw = deg2rad(30);
% hp = plot_box_3D(ax, box_center, box_size, box_yaw, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'FaceAlpha', 0.2, ...
%     'EdgeColor', 'k', ...
%     'EdgeAlpha', 0.4, ...
%     'LineWidth', 1.0, ...
%     'LineStyle', '-', ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5);