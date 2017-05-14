function obj = visualize(obj)
% VISUALIZE     Visualize the @MImage
%
%   See also MIMAGE

% Copyright 2006 Wei-Lwun Lu
% visualize.m version 1.0

visualize(obj.MVisualizable);

if ~isempty(obj.image)
    subimage(obj.image);
else
    warning('No image loaded.');
end
