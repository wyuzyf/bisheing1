function display(obj)
% DISPLAY       Display the @MVisualizable object
%
%   See also MVISUALIZABLE

% Copyright 2006 Wei-Lwun Lu
% display.m version 1.0

disp(sprintf('%s object', class(obj)))
disp(struct(obj));
