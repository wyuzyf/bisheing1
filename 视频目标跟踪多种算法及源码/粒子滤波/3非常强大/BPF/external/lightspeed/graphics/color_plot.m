function color_plot(x,colors)
% COLOR_PLOT    Scatterplot with colored points.
% color_plot(x) makes a scatterplot of x(:,1) versus x(:,2) with points colored
% according to quantiles of x(:,3).
% color_plot(x,n) specifies the number of color quantiles (default 4).
% color_plot(x,colors) specifies an RGB matrix of colors (the number of rows
% determines the number of quantiles).  The default is YlGnBu_colors.
%
% Example:
%   xy = ndgridmat(linspace(-12,12,20),linspace(-12,12,20));
%   z = sin(sqrt(xy(:,1).^2 + xy(:,2).^2));
%   color_plot([xy z]);
%
% See also YlGnBu_colors.

if nargin < 3
  colors = 4;
end
if length(colors) == 1
  nlevels = colors;
  colors = YlGnBu_colors(nlevels);
else
  nlevels = rows(colors);
end
% color groups
c = cut_quantile(x(:,3),nlevels);
for lev = 1:nlevels
  i = find(c == lev);
  plot(x(i,1),x(i,2),'o','Color',colors(lev,:),'MarkerFaceColor',colors(lev,:));
  hold on
end
hold off
set(gca,'Color','none')
