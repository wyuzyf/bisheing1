function val = get(obj, propName)
% GET           Get the properties of the @MIntegralHistogram object
%   val = get(obj, 'propName') get the properties of the 
%   @MIntegralHistogram object. Possible 'propName' includes:
%     'height'      height of the image
%     'width'       width of the image
%     'nbins'       number of bins
%     'indexImage'  2D indexed image whose pixel values represent the bin
%                   number
%     'weightImage' 2D weight image whose pixel values represent the
%                   weights
%
%   Example:
%     load(which('MIntegralHistogram/private/testMat.mat'));
%     obj = MIntegralHistogram(indexImage);
%     val = get(obj, 'nbins');
%
%   See also MINTEGRALHISTOGRAM, SET

%
% Copyright 2006 Wei-Lwun Lu
% get.m version 1.0

switch propName
    % image information
    case 'height'
        val = obj.height;
    case 'width'
        val = obj.width;
    case 'nbins'
        val = obj.nbins;
    case 'indexImage'
        val = obj.indexImage;
    case 'weightImage'
        val = obj.weightImage;
    case 'integralHistogram'
        val = obj.integralHistogram;
        
    otherwise
        error([propName,' Is not a valid @MIntegralHistogram property'])
end
