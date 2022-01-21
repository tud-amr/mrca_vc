% plot an ellipsoid and return the plot handle
% 
% inputs:
%   - ax: figure handle
%   - pos: ellipsoid center, [3]
%   - ell: ellipsoid size, [3]
%   - varargin: patch properties
% 
% outputs: 
%   - h: plot handle
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
% 

function h = plot_ellipsoid_3D(ax, pos, ell, varargin)
    
    % generate plot data
    [X, Y, Z] = ellipsoid(pos(1), pos(2), pos(3), ... 
                ell(1), ell(2), ell(3));

    % plot the ellipsoid
    h = surface(ax, X, Y, Z, varargin{:});
    
end

%% test script
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -3 3 0 2]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% rotate3d(ax);
% view(ax, 3);
% pos = [1; 2; 1];
% ell = [0.5; 0.5; 0.8];
% hell = plot_ellipsoid_3D(ax, pos, ell, ...
%     'FaceColor', 'r', 'FaceAlpha', 0.4, ...
%     'EdgeColor', 'r', 'EdgeAlpha', 0.2);