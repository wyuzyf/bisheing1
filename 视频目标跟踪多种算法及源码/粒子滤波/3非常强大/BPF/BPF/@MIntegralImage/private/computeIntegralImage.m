function integralImage = computeIntegralImage(inputData)
% COMPUTEINTEGRALIMAGE  Compute the integral image
%   integralImage = computeIntegralImage(inputData) computes the 
%   integral image of <inputData> and return an 2D array 
%   <integralImage>.
%
%   See also MINTEGRALIMAGE

% Copyright Wei-Lwun Lu 2006
% computeIntegralImage.m version 1.0

% compute the integral image
integralImage = inputData;
integralImage = cumsum(integralImage, 1);
integralImage = cumsum(integralImage, 2);
