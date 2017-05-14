function obj = setdata(obj, inputData)
% SETDATA       Set the data, width, and height properties of
%               @MIntegralImage
%   obj = setdata(obj, inputData) sets obj.data to inputData, and set
%   obj.width and obj.height accordingly.
%
%   See also MINTEGRALIMAGE, SET, GET

% Copyright 2006 Wei-Lwun Lu
% setdata.m version 1.0

if ndims(inputData) ~= 2
    error('The input data must be a 2D array.');
end

obj.data   = inputData;
obj.height = size(inputData, 1);
obj.width  = size(inputData, 2);
    