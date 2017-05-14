function obj = setweightimage(obj, weightImage)

if size(weightImage) == size(obj.indexImage)
    obj.weightImage = weightImage;
else
    error('Invalid weight image.');
end
