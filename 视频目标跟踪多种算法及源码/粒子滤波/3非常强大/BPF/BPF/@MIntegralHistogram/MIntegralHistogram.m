function obj = MIntegralHistogram(varargin)
% MINTEGRALHISTOGRAM    Constructor of the @MIntegralHistogram object
%
%   Usage: 
%     obj = MIntegralHistogram;
%
%     obj = MIntegralHistogram(indexImage) constructs an integral histogram
%     using a indexed image whose pixel values represent the bin number.
%     The number of bins in 'obj' will be set to the maximum pixel value in
%     the indexed image.
%
%     obj = MIntegralHistogram(indexImage, weightImage) constructs 
%     an integral histogram using a indexed image whose pixel values 
%     represent the bin number, and a weight image whose pixel values 
%     represent the weight.
%
%     obj = MIntegralHistogram(indexImage, weightImage, nbins) constructs 
%     an integral histogram using a indexed image whose pixel values 
%     represent the bin number, and a weight image whose pixel values 
%     represent the weight. The number of bins in 'obj' will be set to 
%     <nbins>. 
%
%   Example:
%     load(which('MIntegralHistogram/private/testMat.mat'));
%     obj = MIntegralHistogram(indexImage);
%     hist1 = gethistogram(obj);
%
%   Reference:
%   [1] F. Porikli, "Integral Histogram: A Fast Way to Extract Histograms
%       in Cartesian Spaces," CVPR 2005.
%
%   See also GET, SET

% Copyright 2006 Wei-Lwun Lu
% MIntegralHistogram.m version 1.0

if nargin == 0
    obj = initFields;
    obj = class(obj, 'MIntegralHistogram');
elseif isa(varargin{1}, 'MIntegralHistogram')
    obj = varargin{1};
else
    obj = initFields;
    obj = class(obj, 'MIntegralHistogram');
    if nargin == 1
        obj = setindeximage(obj, varargin{1});
        obj = setweightimage(obj, ones([obj.height, obj.width]));
        obj = setnbins(obj, max(max(obj.indexImage)));
    elseif nargin == 2
        obj = setindeximage(obj, varargin{1});
        obj = setweightimage(obj, varargin{2});
        obj = setnbins(obj, max(max(obj.indexImage)));
    elseif nargin == 3
        obj = setindeximage(obj, varargin{1});
        obj = setweightimage(obj, varargin{2});
        obj = setnbins(obj, varargin{3});
    else
        error('Too many parameters.');
    end   
    
    obj = computeIntegralHistogram(obj);
end


% -----------------------------------
function obj = initFields()

% public properties
obj.nbins             = 0;
obj.indexImage        = [];
obj.weightImage       = [];

% private properties
obj.width             = 0;
obj.height            = 0;
obj.integralHistogram = [];

