function obj = visualize(obj)
% VISUALIZE     Visualize the @MVisualizable object
%   obj = visualize(obj)
%
%   See also MVISUALIZABLE

% Copyright 2006 Wei-Lwun Lu
% visualize.m version 1.0

if isempty(obj.fighandle)
    obj.fighandle = figure;
end

centerFigure(obj.fighandle, obj.width, obj.height);
