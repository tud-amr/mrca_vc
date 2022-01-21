% plot polyhedron from vertices and return the plot handle
% 
% inputs:
%   - ax: figure handle
%   - v: vertices, [3xN]
% 
% outputs: 
%   - h: plot handle
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
% 

function h = plot_poly_vert_3D(ax, v, varargin)
    
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
% axis([-3 3 -3 3 -2 2]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% rotate3d(ax);
% view(ax, 3);
% v1 = [-1; -1; -1];
% v2 = [ 1; -1; -1];
% v3 = [ 1;  1; -1];
% v4 = [-1;  1; -1];
% v5 = [-1; -1;  1];
% v6 = [ 1; -1;  1];
% v7 = [ 1;  1;  1];
% v8 = [-1;  1;  1];
% v = [v1, v2, v3, v4, v5, v6, v7, v8];
% hbox = plot_poly_vert_3D(ax, v, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'EdgeColor', [0.4 0.4 0.4], ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5, ...
%     'FaceAlpha', 0.6);