% transform a box to polytope verts and linear constraints
% 
% inputs:
%   - dim: problem dimension
%   - verts: box size, length and width, [m*dim]
%   - dist: scalar
% 
% outputs: 
%   - poly_vert: vertice of expanded polytope, [m*dim]
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function poly_vert = expandPolytope(verts, dist)
    
    [A, b] = vert2con(verts);
    m = size(A, 1);
    A_e = A;
    b_e = b;
    for i = 1 : m
        a_e = A_e(i, :);
        b_e(i) = b(i) + dist*norm(a_e);
    end
    poly_vert = con2vert(A_e, b_e);

end
