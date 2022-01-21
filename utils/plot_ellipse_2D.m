% plot an ellipse with orientation and return the plot handle
% 
% inputs:
%   - ax: figure handle
%   - pos: ellipse center, [2x1]
%   - ell: ellipse size, [2x1]
%   - orient: ellipse orientation, 1
%   - varargin: patch properties
% 
% outputs: 
%   - h: plot handle
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
% 

function h = plot_ellipse_2D(ax, pos, ell, orient, varargin)
    
    % generate plot data
    a = ell(1);
    b = ell(2);
    r = 0: 0.1: 2*pi+0.1;
    alpha = [ cos(orient) -sin(orient)
              sin(orient)  cos(orient)];
    p = [(a*cos(r))' (b*sin(r))'] * alpha;
    X = pos(1) + p(:,1);
    Y = pos(2) + p(:,2);

    % plot the ellipse
    h = patch(ax, 'XData', X, 'YData', Y, varargin{:});
    
end

%% test script
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -3 3]);
% ax = hfig.CurrentAxes;
% pos = [1; 2];
% ell = [0.5; 0.8];
% orient = deg2rad(30);
% hell = plot_ellipse_2D(ax, pos, ell, orient, ...
%     'FaceColor', 'r', 'FaceAlpha', 0.4, ...
%     'EdgeColor', 'r', 'EdgeAlpha', 0.2);