% compute a normalized hyperplane to separate two Gaussian distributions
% 
% inputs:
%   - mu1: mean of first Gaussian, [dx1], d is the dimension
%   - cov1: covariance of fist Gaussian, [dxd]
%   - mu2: mean of second Gaussian, [dx1]
%   - cov2: covariance of second Gaussian, [dxd]
% 
% outputs: 
%   - a, b: hyperplane parameters s.t. |a|=1 and a'*mu1 < b, a'*mu2 > b
%   a: [dx1], b: [1]
% 
% (c) Hai Zhu, TU Delft, 2020, h.zhu@tudelft.nl
%

function [a, b] = gaussian_gaussian_hyperplane(mu1, cov1, mu2, cov2)
    
    % solve for t
    t0 = 0.5;
    options = optimoptions('fsolve','Display','off','MaxIterations', 6);
    [t, ~, exitflag] = fsolve(@(x)find_t(x, mu1, cov1, mu2, cov2), t0, options);
    
    if exitflag < 0
        t = 0.5;
    end
    
    a = inv((t*cov1 + (1-t)*cov2)) * (mu2 - mu1);
    b = a'*mu1 + t*a'*cov1*a;
    
    % normalization
    a_norm = norm(a);
    a = a / a_norm;
    b = b / a_norm;

end

function F = find_t(t, mu1, cov1, mu2, cov2)
    
    a = inv((t*cov1 + (1-t)*cov2)) * (mu2 - mu1);
    
    F = a' * (t^2*cov1 - (1-t)^2*cov2) * a;

end

%% test script
%% 2D
% hfig = figure;
% box on;
% grid on;
% axis([-3 3 -3 3]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% hold on;
% dim = [-3 3; -3 3];
% mu1 = [-2; -1];
% cov1 = diag([0.2; 0.1].^2);
% mu2 = [2; 1];
% cov2 = diag([0.1; 0.3].^2);
% plot_error_ellipse_2D(ax, mu1, cov1, 3, ...
%     'LineStyle','--', 'FaceAlpha', 0.0, 'EdgeColor', 'b');
% plot_error_ellipse_2D(ax, mu2, cov2, 3, ...
%     'LineStyle','--', 'FaceAlpha', 0.0, 'EdgeColor', 'b');
% tic
% [a, b] = gaussian_gaussian_hyperplane(mu1, cov1, mu2, cov2);
% toc
% plot_line_2D(ax, a, b, dim, ...
%     'Color', [0.4 0.4 0.4], ...
%     'LineStyle', '--', ...
%     'LineWidth', 1.5);
%% 3D
% hfig = figure;
% box on;
% grid on;
% axis([-4 4 -4 4 -3 3]);
% ax = hfig.CurrentAxes;
% daspect(ax, [1 1 1]);
% rotate3d(ax);
% view(ax, 3);
% hold on;
% mu1 = [-2; -1; -1];
% cov1 = diag([0.2; 0.1; 0.1].^2);
% mu2 = [3; 1; 2];
% cov2 = diag([0.1; 0.3; 0.2].^2);
% plot_error_ellipsoid_3D(ax, mu1, cov1, 3, ...
%     'LineStyle','--', 'FaceAlpha', 0.0, 'EdgeColor', 'b');
% plot_error_ellipsoid_3D(ax, mu2, cov2, 3, ...
%     'LineStyle','--', 'FaceAlpha', 0.0, 'EdgeColor', 'b');
% tic
% [a, b] = gaussian_gaussian_hyperplane(mu1, cov1, mu2, cov2);
% toc
% dim = [-4 4; -4 4; -3 3];
% plot_plane_3D(ax, a, b, dim, ...
%     'FaceColor', [0.4 0.4 0.4], ...
%     'EdgeColor', [0.4 0.4 0.4], ...
%     'SpecularStrength', 0.1, 'AmbientStrength', 0.5, ...
%     'FaceAlpha', 0.6);