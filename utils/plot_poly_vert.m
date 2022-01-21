% plot polyhedron from vertices and return the plot handle
% 
% inputs:
%   - ax: figure handle
%   - dim: dimension, d
%   - v: vertices, [dxN]
% 
% outputs: 
%   - h: plot handle
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
% 

function h = plot_poly_vert(ax, dim, v, varargin)
    
    if dim == 2
        h = plot_poly_vert_2D(ax, v, varargin{:});
    elseif dim == 3
        h = plot_poly_vert_3D(ax, v, varargin{:});
    else
        error('Dimension error!');
    end
    
end
