function [hf, hp, d_min, x_d_min, y_d_min, is_vertex, idx_c, xc, yc, is_in_seg, Cer, Ppr] = test_p_poly_dist(xp, yp, xv, yv, varargin)
%test_p_poly_dist Plot the results of a call to p_poly_dist function
% Input:
% xp - vector of points X coordinates (1 X np)
% yp - vector of points Y coordinates (1 X np)
% xv - vector of polygon vertices' X coordinates (1 X nv)
% yv - vector of polygon vertices' Y coordinates (1 X nv)
% varargin - list of optional arguments to p_poly_dist
% Output:
% hf - figure handle
% hp - plot handles
% d_min, x_d_min, y_d_min, is_vertex, idx_c, xc, yc, is_in_seg, Cer, Ppr - 
%    p_poly_dist output arguments
%
% Usage example:
% xp = [-1 0 2];
% yp = [-3 -2 1];
% 
% xv = [-3 -1 0 1 2 2 0];
% yv = [-1 0 1 1 -1 -2 -3];
%
% For distances to polyline:
% [hf, hp, d_min, x_d_min, y_d_min, is_vertex, idx_c, xc, yc, is_in_seg, Cer, Ppr] = test_p_poly_dist(xp, yp, xv, yv)
%
% For distances to closed polygon:
% [hf, hp, d_min, x_d_min, y_d_min, is_vertex, idx_c, xc, yc, is_in_seg, Cer, Ppr] = test_p_poly_dist(xp, yp, xv, yv, true)

nv = length(xv);
np = length(xp);

if(nargin>4)
   find_in_out = varargin{1};
   if(find_in_out)
      % If (xv,yv) is not closed, close it.
      nv = length(xv);
      if ((xv(1) ~= xv(nv)) || (yv(1) ~= yv(nv)))
         xv = [xv(:)' xv(1)];
         yv = [yv(:)' yv(1)];
         nv = nv + 1;
      end
   end
end

hf = figure;
grid on;
axis equal;
hold on

[d_min, x_d_min, y_d_min, is_vertex, idx_c, xc, yc, is_in_seg, Cer, Ppr] = ...
     p_poly_dist(xp, yp, xv, yv, varargin{:});

% vertices
hp(1) = plot(xv, yv, 'rs');

% segments
hp(2) = plot(xv, yv, 'b-', 'Linewidth', 2);

% closest points
hp(3) = ...
   plot(x_d_min, y_d_min, 'ro', 'MarkerFaceColor', 'r');

% points
hp(4) = plot(xp, yp, 'mx', 'MarkerSize', 10, 'Linewidth', 2);

vtmp = plot(xc, yc, 'go');
nc = length(vtmp);
hp(5:4+nc) = vtmp;

% text offsets
off_x = 0.1;
off_y = 0.2;

seg_ex_plotted = false(np, nv-1);
for j=1:(nv-1),
   % plot projected intersection points
   ht=text(xv(j)-off_x, yv(j)-off_y, int2str(j));
   for k=1:np, 
      str = ['(' num2str(k) ',' num2str(j) ')'];
      str_p{k} = int2str(k);
      ht=text(xc(k,j)+off_x, yc(k,j)+off_y, str);
      % segment extensions beyond the verices
      if(~is_in_seg(k,j) && (~seg_ex_plotted(k,j)))
         hp(end+1) =  plot([xv(j+1), xc(k,j)], [yv(j+1) yc(k,j)], 'g--');
         seg_ex_plotted(k,j) = true;
      end
   end
end
htp=text(xp-off_x, yp-off_y, str_p);   

xlabel('X');
ylabel('Y');
legend(hp([1:5, 5+nc]), {'vertices', 'segments', 'closest points', 'points', 'intersection points', 'segment extensions'}, 'Location', 'EastOutside');

end

