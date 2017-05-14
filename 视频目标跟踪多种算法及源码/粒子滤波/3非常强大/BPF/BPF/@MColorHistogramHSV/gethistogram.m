function [hshist, vhist] = gethistogram(obj, varargin)
% GETHISTOGRAM  Get the color histogram
%   [hshist, vhist] = gethistogram(obj) gets the HS-channel histogram and
%   V-channel histogram of the entire image.
%
%   [hshist, vhist] = gethistogram(obj, rect) gets the HS-channel histogram
%   and V-channel histogram histogram of region defined in <rect>. 
%   <rect> = [x1, y1, x2, y2] where (x1, y1) is the inclusive upper-left 
%   corner of the region, and (x2, y2) is the inclusive lower-right corner 
%   of the region.
%
%   [hshist, vhist] = gethistogram(obj, rect, 1) gets the color histogram 
%   of region defined in <rect>, and normalizes the histogram to make it 
%   sum to 1.
%
%   Example:
%     obj = MColorHistogramHSV('moon.tif');
%     [hshist, vhist] = gethistogram(obj);
%
%   See also MCOLORHISTOGRAMHSV, GET, SET

% Copyright 2006 Wei-Lwun Lu
% gethistogram.m version 1.0

switch obj.type
    case 'global'
        hshist = gethistogram(obj.hsIntegralHistogram, varargin{:});
        vhist  = gethistogram(obj.vIntegralHistogram, varargin{:});
    case 'local'
        [hshist, vhist] = gethistogram_local(obj, varargin{:});
    case 'builtin'
        [hshist, vhist] = gethistogram_builtin(obj, varargin{:});
    case 'otherwise'
        error('Unknown type.');
end


