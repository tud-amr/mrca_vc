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

function h = plot_box_2D(ax, box_pos, box_size, box_yaw, varargin)

    % convert to verts
    [c, ~] = box2PolyVertsCons_2D(box_pos, box_size, box_yaw);

    if isempty(c)
      return
    end
    
    k = convhull(c(1,:), c(2,:));
    X = reshape(c(1,k'), size(k'));
    Y = reshape(c(2,k'), size(k'));
    h = patch(ax, X,Y, 'k', varargin{:});
    
end

%% test script
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -2 2]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% hold on;
% box_center = [0.5; 1];
% box_size = [2; 1];
% box_yaw = deg2rad(30);
% hp = plot_box_2D(ax, box_center, box_size, box_yaw, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'FaceAlpha', 0.2, ...
%     'EdgeColor', 'k', ...
%     'EdgeAlpha', 0.4, ...
%     'LineWidth', 1.0, ...
%     'LineStyle', '-', ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5);