function [A_bound, b_bound] = boundsCon(dim, lb, ub)

    % A_bound: m*dim
    % b_bound: m*1

    if dim == 2                   % 2D
        A_bound = [-1,  0; 
                    0, -1;
                    1,  0;
                    0,  1];
        b_bound = [-lb; ub];
    elseif dim == 3               % 3D
        A_bound = [-1,  0,  0;
                    0, -1,  0;
                    0,  0, -1;
                    1,  0,  0;
                    0,  1,  0;
                    0,  0,  1];
        b_bound = [-lb; ub];
    end

end
