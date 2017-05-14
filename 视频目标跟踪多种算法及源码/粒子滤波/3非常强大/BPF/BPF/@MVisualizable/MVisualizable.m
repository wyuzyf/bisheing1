function obj = MVisualizable(varargin)
% MVISUALIZABLE  Constructor of the @MVisualizable class
%   obj = MVisualizable;
%   obj1 = MVisualizable(obj2);
%   obj = MVisualizable(width, height, title);
%
%   See also GET, SET

% Copyright 2006 Wei-Lwun Lu
% MVisualizable.m version 1.0

if nargin == 0
    obj = initFields;
    obj = class(obj, 'MVisualizable');
elseif isa(varargin{1}, 'MVisualizable')
    obj = varargin{1};
else
    obj = initFields;
    obj = class(obj, 'MVisualizable');
    if nargin == 1
        obj.width = varargin{1};
    elseif nargin == 2
        obj.width  = varargin{1};
        obj.height = varargin{2};
    elseif nargin == 3
        obj.width  = varargin{1};
        obj.height = varargin{2};
        obj.title  = varargin{3};
    end
end


% -----------------------------------
function obj = initFields()

% general
obj.width  = 640;
obj.height = 480;
obj.title  = 'empty figure';

obj.fighandle = [];
