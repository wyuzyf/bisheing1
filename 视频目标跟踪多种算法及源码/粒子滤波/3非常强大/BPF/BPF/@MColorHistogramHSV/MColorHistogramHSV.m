function obj = MColorHistogramHSV(varargin)
% MCOLORHISTOGRAMHSV   Constructor of the @MColorHistogramHSV
%   This class compute the color histogram of a gray image.
%
%   Usage: 
%      obj = MColorHistogramHSV creates an empty MColorHistograyHSV.
%
%      obj = MColorHistogramHSV(mimage) creates a MColorHistograyHSV from
%      a MImage object <mimage>.
%
%      obj = MColorHistogramHSV(mimage, nbins) creates a <nbins>-bins
%      MColorHistogramHSV from a MImage object <mimage>.
%      
%      obj = MColorHistogramHSV(mimage, nbins, type) creates a <nbins>-bins
%      MColorHistogramHSV from a MImage object <mimage>. Possible type
%      includes 'global', 'local', and 'builtin'. See GET for more details.
%
%   Example:
%      obj = MColorHistogramHSV('moon.tif');
%      hist1 = gethistogram(obj);
%
%   See also GET, SET

% Copyright 2006 Wei-Lwun Lu
% MColorHistogramHSV.m version 1.0

if nargin == 0
    obj = initFields;
    obj = class(obj, 'MColorHistogramHSV', MImage, MVisualizable);
elseif isa(varargin{1}, 'MColorHistogramHSV')
    obj = varargin{1};
else
    obj = initFields;
    
    if nargin == 0
        obj = class(obj, 'MColorHistogramHSV', MImage, MVisualizable);
    elseif nargin == 1
        obj = class(obj, 'MColorHistogramHSV', MImage, MVisualizable);
        obj = setimage(obj, MImage(varargin{1}));
    elseif nargin == 4
        obj = class(obj, 'MColorHistogramHSV', MImage, MVisualizable);
        obj.hbins  = varargin{2};
        obj.sbins  = varargin{3};
        obj.vbins  = varargin{4};
        obj.hsbins = varargin{2} * varargin{3};
        obj = setimage(obj, MImage(varargin{1}));
    elseif nargin == 5
        obj = ...
            class(obj, 'MColorHistogramHSV', MImage, MVisualizable);
        obj.hbins  = varargin{2};
        obj.sbins  = varargin{3};
        obj.vbins  = varargin{4};
        obj.hsbins = varargin{2} * varargin{3};
        obj.type   = varargin{5};
        obj = setimage(obj, MImage(varargin{1}));
    else
        error('Invalid number of arguments.');
    end
    
    obj = computeCache(obj);
end


% -----------------------------------
function obj = initFields()

% fields of the HOG descriptor
obj.hbins = 10;
obj.sbins = 10;
obj.vbins = 10;
obj.type  = 'builtin';

% private fields
obj.width = 0;
obj.height = 0;
obj.data   = [];
obj.hsbins = 100;
obj.hsIntegralHistogram = [];
obj.vIntegralHistogram = [];
