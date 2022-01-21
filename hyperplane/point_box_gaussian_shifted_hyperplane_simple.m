% compute a normalized max-margin shifted separating hyperplane between 
% a point and a box obstacle with uncertain position
% 
% inputs:
%   - p: point 1, [dx1], d is the dimension
%   - pt: box center, [dx1]
%   - pt_cov: box pos cov, [dxd]
%   - size: box size, [dx1]
%   - yaw: box yaw, [1]
%   - delta: collision probability threshold
% 
% outputs: 
%   - a, b: hyperplane parameters s.t. |a|=1 and a'*p < b, a'*vert(:,i) > b
%   a: [dx1], b: [1]
% 
% note: using the SVM maximum margin classifier method
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [a, b] = point_box_gaussian_shifted_hyperplane_simple(p, po, po_cov, size_o, yaw_o, delta)

    d = length(p);      % dimension

    % coordinate transformation
    W = zeros(d, d);
    l = po_cov(1,1);                         	% if cov is diagnal and 
    for i = 1 : d
        W(i, i) = 1 / (sqrt(l));
    end
    scale = 1 / (sqrt(l));
    
    % transform the point
    p_t = W*p;
    
    % perform transformation of the box
    po_t = W*po;                                % the center
    size_t = scale*size_o;
    yaw_t = yaw_o;
    
    % tranform to polyAb
    if d == 2
        [~, poly_Ab] = box2PolyVertsCons_2D(po_t, size_t, yaw_t);   % 3x4
    elseif d == 3
        [~, poly_Ab] = box2PolyVertsCons_3D(po_t, size_t, yaw_t);   % 4x8
    else
        error('Dimension error!')
    end
    
    % compute obstacle shadow
    eps = sqrt(delta);
    r_square = icdf('Chisquare', 1-eps, d);
    r = sqrt(r_square);
    
    % expanding
    A_t = poly_Ab(1:d, :)';
    b_t = poly_Ab(d+1, :)';
    m = size(A_t, 1);
    A_e = A_t;
    b_e = b_t;
    for i = 1 : m
        a_e = A_e(i, :);
        b_e(i) = b_t(i) + 2*r*norm(a_e);
    end
    
    % convert to vert
    vert_t_expand = con2vert(A_e, b_e);

    % compute the max-margin hyperplane in transformed space
    [a_t, b_t] = point_polytope_shifted_hyperplane(p_t, vert_t_expand');
    
    % inverse transformation
    a = W' * a_t;
    b = b_t;   
    
    % normalization
    a_norm = norm(a);
    a = a ./ a_norm;
    b = b ./ a_norm;

end
