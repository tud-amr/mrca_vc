% transform a box to polytope verts and linear constraints
% 
% inputs:
%   - dim: dimension
%   - pos: box center position, x and y, [dx1]
%   - size: box size, length and width, [dx1]
%   - orientation_xy: box orientation, [1]
% 
% outputs: 
%   - poly_vert: vertice represented the box, d*m
%   - poly_Ab: linear constraints represented half planes of the box, (d+1)*m
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [poly_vert, poly_Ab] = box2PolyVertsCons(dim, pos, size, orientation_xy)
    
    if dim == 2             % 2D
        [poly_vert, poly_Ab] = box2PolyVertsCons_2D(pos, size, orientation_xy);
    elseif dim == 3         % 3D
        [poly_vert, poly_Ab] = box2PolyVertsCons_3D(pos, size, orientation_xy);
    else
        error('Dimension error!')
    end

end
