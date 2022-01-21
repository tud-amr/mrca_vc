% plot a line in 2D and return the plot handle
% 
% inputs:
%   - ax: figure handle
%   - a: line para, [2x1]
%   - b: line para, [1], s.t. a*x = b
%   - dim: environment size, [2x2], [xmin, xmax; ymin, ymax]
%   - varargin: line properties
% 
% outputs: 
%   - h: plot handle
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function h = plot_line_2D(ax, a, b, dim, varargin)

    % find points on the line
    points = zeros(2, 2);
    
    if a(2) == 0            % a vertical line, x=b
        points(:, 1) = [b; dim(2,1)];
        points(:, 2) = [b; dim(2,2)];
    else
        points(1, 1) = dim(1,1);
        points(2, 1) = (b - a(1)*points(1,1)) / a(2);
        points(1, 2) = dim(1,2);
        points(2, 2) = (b - a(1)*points(1,2)) / a(2);
    end
    
    % plot the line
    h = line(ax, points(1, :), points(2, :), varargin{:});

end

%% test script
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -2 2]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% hold on;
% a = [1 0];
% b = -1;
% dim = [-3 3; -2 2];
% hp = plot_line_2D(ax, a, b, dim, ...
%     'Color', [0.4 0.4 0.4], ...
%     'LineStyle', '--', ...
%     'LineWidth', 1.5);