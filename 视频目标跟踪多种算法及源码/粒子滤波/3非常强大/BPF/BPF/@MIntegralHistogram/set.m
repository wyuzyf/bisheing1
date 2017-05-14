function obj = set(obj, varargin)
% SET           Set properties of the @MIntegralHistogram object
%   obj = set(obj, 'propName', val, ...) sets the properties of the
%   @MIntegralHistogram object. Possible 'propName' includes:
%     'nbins'       number of bins
%     'indexImage'  2D indexed image whose pixel values represent the bin
%                   number
%     'weightImage' 2D weight image whose pixel values represent the
%                   weights
%
%
%   Example:
%     load(which('MIntegralHistogram/private/testMat.mat'));
%     obj = MIntegralHistogram(indexImage);
%     obj = set(obj, 'nbins', 13);
%
%   See also MINTEGRALHISTOGRAM, GET

% Copyright 2006 Wei-Lwun Lu
% set.m version 1.0

propertyArgIn = varargin;
while length(propertyArgIn) >= 2,
    prop = propertyArgIn{1};
    val = propertyArgIn{2};
    propertyArgIn = propertyArgIn(3:end);
    switch prop
        % image info
        case 'indexImage'
            obj = setindeximage(obj, val);
            obj = computeIntegralHistogram(obj);
        case 'nbins'
            obj = setnbins(obj, val);
            obj = computeIntegralHistogram(obj);
        case 'weightImage'
            obj = setweightimage(obj, val);
            obj = computeIntegralHistogram(obj);
    end
end

