% compute a normalized max-margin shifted separating hyperplane between 
% a point and a box obstacle
% 
% inputs:
%   - p: point 1, [dx1], d is the dimension
%   - pt: box center, [dx1]
%   - size: box size, [dx1]
%   - yaw: box yaw, [1]
% 
% outputs: 
%   - a, b: hyperplane parameters s.t. |a|=1 and a'*p < b, a'*vert(:,i) > b
%   a: [dx1], b: [1]
% 
% note: using the SVM maximum margin classifier method
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [a, b] = point_box_shifted_hyperplane(p, pt, size, yaw)
    
    % convert box to verts
    dim = length(p);
    
    if dim == 2
        [poly_vert, ~] = box2PolyVertsCons_2D(pt, size, yaw);
    elseif dim == 3
        [poly_vert, ~] = box2PolyVertsCons_3D(pt, size, yaw);
    else
        error('Dimension error!')
    end

    % compute the hyperplane
    [a, b] = point_polytope_shifted_hyperplane(p, poly_vert);
    
end
