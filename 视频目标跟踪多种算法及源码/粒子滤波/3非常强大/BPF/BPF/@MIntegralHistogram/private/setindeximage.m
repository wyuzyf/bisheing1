function obj = setindeximage(obj, indexImage)
% SETINDEXIMAGE
%   obj = setindeximage(obj, indexImage) sets the indexImage property of
%   @MIntegralHistogram.
%

obj.indexImage = indexImage;
obj.width  = size(obj.indexImage, 2);
obj.height = size(obj.indexImage, 1); 

maxIndex = max(max(indexImage));
if maxIndex > obj.nbins
    obj.nbins = maxIndex;
end

if size(obj.indexImage) ~= size(obj.weightImage)
    obj.weightImage = ones([obj.height, obj.width]);
end
