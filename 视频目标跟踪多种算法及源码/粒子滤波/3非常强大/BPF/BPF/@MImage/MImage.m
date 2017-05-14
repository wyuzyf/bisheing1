function obj = MImage(varargin)
% MIMAGE        Constructor of @MImage
%   obj = MImage;
%   obj = MImage(filename);
%   obj = MImage(imageMatrix);
%
%   Example:
%       obj = MImage('peppers.png');
%       visualize(obj);
%
%   See also GET, SET, VISUALIZE

% Copyright 2006 Wei-Lwun Lu
% MImage.m version 1.0

if nargin == 0
    obj = initFields;
    obj = class(obj, 'MImage', MVisualizable);
elseif isa(varargin{1}, 'MImage')
    obj = varargin{1};
else
    obj = initFields;
    obj = class(obj, 'MImage', MVisualizable);
    
    if ischar(varargin{1})
        obj = imread(obj, varargin{1});
    else
        obj = set(obj, 'image', varargin{1});
    end
end


% -----------------------------------
function obj = initFields()

% image info
obj.height       = 0;
obj.width        = 0;
obj.isColorImage = 0;
obj.image        = [];
