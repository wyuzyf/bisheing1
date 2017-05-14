function obj = MIntegralImage(varargin)
% MINTEGRALIMAGE    Constructor of @MIntegralImage
%   obj = MIntegralImage.
%
%   obj = MIntegralImage(inputData) initializes <obj> using a 2D array
%   <inputData>.
%
%   Examples:
%       obj = MIntegralImage(magic(10));
%       val = getregion(obj, [3, 3, 5, 5]);
%
%   See also GET, SET

% Copyright 2006 Wei-Lwun Lu
% MIntegralImage.m version 1.0

if nargin == 0
    obj = initFields;
    obj = class(obj, 'MIntegralImage');
elseif isa(varargin{1}, 'MIntegralImage')
    obj = varargin{1};
else
    obj = initFields;
    obj = class(obj, 'MIntegralImage');
    
    inputData = varargin{1};
    if ndims(inputData) ~= 2
        error('The input data must be a 2D array.');
    end
    obj.height = size(inputData, 1);
    obj.width  = size(inputData, 2);
    
    obj.integralImage = computeIntegralImage(inputData);
end


% -----------------------------------
function obj = initFields()

obj.integralImage = [];
obj.width         = 0;
obj.height        = 0;

