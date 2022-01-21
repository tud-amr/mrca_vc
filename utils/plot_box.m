% plot a box represented in 2D
% 
% inputs:
%   - ax: figure handle
%   - dim: dimension
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

function h = plot_box(ax, dim, box_pos, box_size, box_yaw, varargin)

    if dim == 2
        h = plot_box_2D(ax, box_pos, box_size, box_yaw, varargin{:});
    elseif dim == 3
        h = plot_box_3D(ax, box_pos, box_size, box_yaw, varargin{:});
    else
        error('Dimension error!');
    end
    
end
