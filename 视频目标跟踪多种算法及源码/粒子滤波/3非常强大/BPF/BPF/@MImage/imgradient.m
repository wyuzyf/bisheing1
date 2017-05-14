function [gx, gy] = imgradient(obj, varargin)
% IMGRADIENT    Compute the image gradient of the @MImage
%   [gx, gy] = imgradient(MImage, varargin) computes the image gradients 
%   of @MImage.
%
%   varargin:
%       'kernel'        the kernel used to compute the image gradient
%                       {'11'} | '101' | '18081'
%       'ColorProc'     the way to compute image gradient of color image
%                       {'mean'} | 'max'
%
%   See also MIMAGE

% Copyright 2006 Wei-Lwun Lu
% imgradient.m version 1.1

% process parameters
[gradientType, colorProc, isNormalized] = process_options(varargin, ...
    'kernel', '101', 'ColorProc', 'mean', 'normalized', 0);

switch gradientType
    case '11'
        xConvWindow = [-1, 1];
        yConvWindow = [-1; 1];
    case '101'
        xConvWindow = [-1, 0, 1];
        yConvWindow = [-1; 0; 1];
    case '18081'
        xConvWindow = [1, -8, 0, 8, -1];
        yConvWindow = [1; -8; 0; 8; -1];
    otherwise
        error('Invalid kernel.');
end

imageMatrix = get(obj, 'image');

if isNormalized
    if ndims(imageMatrix) == 2
        imageMatrix = imageMatrix / sum(sum(imageMatrix));
    else
        for n = 1 : size(imageMatrix, 3)
            imageMatrix(:,:,n) = imageMatrix(:,:,n) / sum(sum(imageMatrix(:,:,n)));
        end
    end
end

% compute image gradient (required Image Processing Toolbox)
gxMatrix  = imfilter(imageMatrix, xConvWindow, 'replicate');
gyMatrix  = imfilter(imageMatrix, yConvWindow, 'replicate');

switch colorProc
    case 'mean'
        gxMatrix = mean(gxMatrix, 3);
        gyMatrix = mean(gyMatrix, 3);
    case 'max'
        gxMatrix = max(gxMatrix, [], 3);
        gyMatrix = max(gyMatrix, [], 3);
    otherwise
        error('Invalid ColorProc.');
end

gx = MImage(gxMatrix);
gy = MImage(gyMatrix);



