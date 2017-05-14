function obj = subimage(obj, rect)
% SUBIMAGE      Get subimage
%   obj = subimage(obj, rect) returns a subimage of obj by specifying a
%   rectangle region rect = [x1, y1, x2, y2] where [x1, y1] is the
%   coordinate of the upper-left corner and [x2, y2] is the lower-right
%   corner inclusively.
%
%   Example:
%     obj1 = MImage('tree.tif');
%     obj2 = subimage(obj1, [100, 100, 200, 200]);
%
%   See also: MIMAGE

% Copyright 2006 Wei-Lwun Lu
% subimage.m version 1.0

rect(1) = max(rect(1), 1);
rect(2) = max(rect(2), 1);
rect(3) = min(rect(3), obj.width);
rect(4) = min(rect(4), obj.height);

if obj.isColorImage
    obj = MImage(obj.image(rect(2):rect(4), rect(1):rect(3), :));
else
    obj = MImage(obj.image(rect(2):rect(4), rect(1):rect(3)));
end
