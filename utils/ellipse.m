function [X, Y] = ellipse(pos, ell, yaw)

    % generate data
    a = ell(1);
    b = ell(2);
    r = 0: 0.1: 2*pi+0.1;
    alpha = [ cos(yaw) -sin(yaw)
              sin(yaw)  cos(yaw)];
    p = [(a*cos(r))' (b*sin(r))'] * alpha;
    X = pos(1) + p(:,1);
    Y = pos(2) + p(:,2);

end
