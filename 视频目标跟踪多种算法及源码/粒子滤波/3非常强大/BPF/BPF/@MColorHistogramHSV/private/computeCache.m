function obj = computeCache(obj)
% COMPUTECACHE  Compute the cache of the color histogram
%   obj = computeCache(obj) computes the cache of the color histogram
%
%   See also MCOLORHISTOGRAMGRAY, GET, SET, GETHISTOGRAM

% Copyright 2006 Wei-Lwun Lu
% computeCache.m version 1.0

switch obj.type
    case 'global'
        obj = computeIntegralHistogram(obj);
    case 'local'
        
    case 'builtin'
        
    otherwise
        error('Unknown type.');
end


function obj = computeIntegralHistogram(obj)

% get properties of the color histogram
img     = obj.data;
width   = size(img, 2);
height  = size(img, 1);
hbins   = obj.hbins;
sbins   = obj.sbins;
vbins   = obj.vbins;
hsbins  = obj.hsbins; 

% compute the integral image
[hsImage, vImage] = computeIndexImage(img, hbins, sbins, vbins);

obj.hsIntegralHistogram = ...
    MIntegralHistogram(hsImage, ones([height, width]), hsbins);
obj.vIntegralHistogram = ...
    MIntegralHistogram(vImage, ones([height, width]), vbins);
