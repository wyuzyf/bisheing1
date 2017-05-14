function obj = set(obj, varargin)
% SET           Set properties of the @MIntegralImage object
%   obj = set(obj, 'data', val)
%
%   Example:
%       obj = MIntegralImage;
%       obj = set(obj, 'data', pascal(10));
%
%   See also MINTEGRALIMAGE, GET

% Copyright 2006 Wei-Lwun Lu
% set.m version 1.0

propertyArgIn = varargin;
while length(propertyArgIn) >= 2,
    prop = propertyArgIn{1};
    val = propertyArgIn{2};
    propertyArgIn = propertyArgIn(3:end);
    switch prop
        % image info
        case 'data'
            inputData = val;
    
            if ndims(inputData) ~= 2
                error('The input data must be a 2D array.');
            end
            obj.height = size(inputData, 1);
            obj.width  = size(inputData, 2);
            
            obj.integralImage = computeIntegralImage(inputData);
    end
end
