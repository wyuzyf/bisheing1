function val = getregion(obj, rect)
% GETREGION     Get the sum of a region of the image
%   val = getregion(obj, rect) returns the sum of a region defined in
%   <rect> = [x1, y1, x2, y2].
%
%   Example:
%     obj = MIntegralImage(pascal(10));
%     val = getregion(obj, [2, 2, 5, 5]);
%
%   See also MINTEGRALIMAGE, GET, SET

% Copyright 2006 Wei-Lwun Lu
% getregion.m version 1.0

% process the arguments
obj = struct(obj);
if nargin == 1
    rect = [1, 1, obj.width, obj.height];
elseif nargin == 2
    if rect(1) < 1 || rect(2) < 1 || ...
       rect(3) > obj.width || rect(4) > obj.height
        error('Required region is out of bound.');
    end
else
    error('Invalid number of arguments.');
end


nRect = size(rect, 1);
integralImage = obj.integralImage;
val = zeros([1, nRect]);
for iRect = 1 : nRect
    x1 = rect(iRect, 1);
    y1 = rect(iRect, 2);
    x2 = rect(iRect, 3);
    y2 = rect(iRect, 4);
    
    if x1 > 1 && y1 > 1
        % general case
        val(iRect) =   integralImage(y2  , x2  ) ...
                     - integralImage(y1-1, x2  ) ...
                     - integralImage(y2  , x1-1) ...
                     + integralImage(y1-1, x1-1);
    elseif x1 > 1 && y1 == 1
        val(iRect) =   integralImage(y2, x2  ) ...
                     - integralImage(y2, x1-1);
    elseif x1 == 1 && y1 > 1
        val(iRect) =   integralImage(y2  , x2) ...
                     - integralImage(y1-1, x2);
    else
        val(iRect) = integralImage(y2, x2);
    end
end
