function [hshist, vhist] = gethistogram_builtin(obj, varargin)
    
switch nargin
    case 1
        rect = [1, 1, obj.width, obj.height];
        isNormalized = 0;
    case 2
        rect = varargin{1};
        isNormalized = 0;
    case 3
        rect = varargin{1};
        isNormalized = varargin{2};
    otherwise
        error('Incorrect number of arguments');
end

if isempty(rect)
    rect = [1, 1, obj.width, obj.height];
end

hsbins = obj.hsbins;
hbins = obj.hbins;
sbins = obj.sbins;
vbins = obj.vbins;
img   = obj.data;
nRect = size(rect, 1);
    
[hsImage, vImage] = computeIndexImage(img, hbins, sbins, vbins);

hshist = zeros([nRect, hsbins]);
vhist  = zeros([nRect, vbins]);
for iRect = 1 : nRect
    x1 = rect(iRect, 1);
    y1 = rect(iRect, 2);
    x2 = rect(iRect, 3);
    y2 = rect(iRect, 4);
    
    for y = y1 : y2
        for x = x1 : x2
            hsindex = hsImage(y, x);
            vindex  = vImage(y, x);
            hshist(iRect, hsindex) = hshist(iRect, hsindex) + 1;
            vhist(iRect, vindex)   = vhist(iRect, vindex) + 1;
        end
    end
end

    
if isNormalized
    hshist = makeStochastic(hshist);
    vhist  = makeStochastic(vhist);
end
