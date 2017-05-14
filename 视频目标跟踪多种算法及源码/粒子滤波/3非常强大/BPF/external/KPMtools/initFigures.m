function initFigures(N)

if nargin == 0
  N= 6;
end

% initFigures
% Position 6 figures on the edges of the screen.
% [xmin ymin w h] where (0,0) = bottom left
% Numbers assume screen resolution is 1024 x 1280

global FIGNUM NUMFIGS
FIGNUM = 1; NUMFIGS = N;

screenMain = true; % set to false if initializing figures for second screen
%screenMain = false;

if screenMain
  xoff = 0; 
else
  %xoff = 1280;
  xoff = -1280;
end

switch N
 case 6,
  % 2 x 3 design
  w = 400; h = 300;
  xs = [10 450 875] + xoff;
  ys = [650 40];
 case 9,
  % 3x3 design
  w = 350; h = 250;
  xs = [10 380 750]+xoff;
  ys = [700 350 10];
 case 12,
  % 3x4
  w = 300; h = 270; dw = 20; dh = 50;
  nc = 4; nr = 3;
  xs(1) = 10;
  for c=2:nc
    xs(c) = xs(c-1) + w + dw + xoff;
  end
  ys(1) = 1024-h-2*dh;
  for r=2:nr
    ys(r) = ys(r-1) - h - dh;
  end
end
 


Nfigs = length(xs)*length(ys);
if screenMain
  fig = 1; 
else
  fig = Nfigs + 1;
end

for yi=1:length(ys)
  for xi=1:length(xs)
    figure(fig);
    set(gcf, 'position', [xs(xi) ys(yi) w h]);
    fig = fig + 1;
  end
end

% To plot something on the next available figure (with wrap around), use
% sfigure(FIGNUM); clf; FIGNUM = wrap(FIGNUM+1, NUMFIGS); 
