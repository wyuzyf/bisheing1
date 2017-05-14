function display(obj)
% DISPLAY       Display the @MImage object
%
%   See also MIMAGE

% Copyright 2006 Wei-Lwun Lu
% display.m version 1.0

disp(sprintf('%s object', class(obj)))
disp(struct(obj));
