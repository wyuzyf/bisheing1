function [histogram, upper, lower, bound] = compute_HSVhist(hsv_obj, frame, numBox)
% PURPOSE : Get the hsv histogram 
% INPUT : - hsv_obj = hsv color object
%         - frame    = tracking region
%         - numBox     = the number of histograms 
% OUTPUT : - histogram = color histogram extracted from the tracked region
%          - upper     = bin index values of upper body when numBox = 2
%          - lower     = bin index values of lower body when numBox = 2
%          - bound     = boundary of the tracked region [x1 y1 x2 y2]
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

left = frame(1); 
top = frame(2);
right = frame(3);
bottom = frame(4);

% color histogram is computed in a square box centered on the origial rectangle box
rectWidth = right - left;
rectHeight = bottom - top;
widthOffset = round(rectWidth/4.0);
heightOffset = round(rectHeight/8.0);
left = left + widthOffset;
top = top + heightOffset;
right = right - widthOffset;
bottom = bottom - heightOffset;

upper = 0;
lower = 0;
if numBox == 1
    [HShist Vhist] = gethistogram(hsv_obj, round([left top right bottom]), 1);
    histogram.HS = reshape(HShist, [10 10]);
    histogram.V = Vhist';
elseif numBox == 2
    divider = round((bottom - top)/2 + top);
    % upper
    [HShist Vhist] = gethistogram(hsv_obj, round([left top right divider]), 1);
    histogram.HS(:,:,1) = reshape(HShist, [10 10]);
    histogram.V(:,1) = Vhist';
    % lower
    [HShist Vhist] = gethistogram(hsv_obj, round([left divider right bottom]), 1);
    histogram.HS(:,:,2) = reshape(HShist, [10 10]);
    histogram.V(:,2) = Vhist';
end

bound = [left top right bottom];
