% plot a polytope represented by a set of vertice in 2D
% 
% inputs:
%   - ax: figure handle
%   - c: set of vertice, [2xn]
%   - varargin: patch properties
% 
% outputs: 
%   - h: plot handle
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
% 

function h = plot_poly_vert_2D(ax, c, varargin)

    if isempty(c)
        h = plot(polyshape());
        return
    end
    
    try 
        k = convhull(c(1,:), c(2,:));
        X = reshape(c(1,k'), size(k'));
        Y = reshape(c(2,k'), size(k'));
        h = patch(ax, X,Y, 'k', varargin{:});
    catch
        h = [];
    end
    
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
% box_vert = box2PolyVertsCons_2D(box_center, box_size, box_yaw);
% hp = plot_poly_vert_2D(ax, box_vert, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'FaceAlpha', 0.2, ...
%     'EdgeColor', 'b', ...
%     'EdgeAlpha', 0.6, ...
%     'LineWidth', 1.0, ...
%     'LineStyle', '-.', ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5);