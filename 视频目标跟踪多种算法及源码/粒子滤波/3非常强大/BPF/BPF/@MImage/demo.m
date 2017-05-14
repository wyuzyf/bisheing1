function obj = demo(obj)
% DEMO          Demo the @MImage class
%
%   See also MIMAGE

% Copyright 2006 Wei-Lwun Lu
% demo.m version 1.0

obj = MImage(which('MImage/private/peppers.png'));
visualize(obj);
