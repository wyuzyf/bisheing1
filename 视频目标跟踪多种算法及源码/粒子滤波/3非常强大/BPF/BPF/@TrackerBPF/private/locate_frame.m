function frame = locate_frame(obj, center, height, width)
% PURPOSE : Get the location of the frame (i.e, four corners)
% INPUT : - obj    = the @TrackerBPF object
%         - center = the center position of the tracked region
%         - height = height of the region
%         - width  = width of the region
% OUTPUT : - frame = located frame
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

height = round(height/2);
width = round(width/2);

left = max(1, center(1) - width);
top = max(1, center(2) - height);
right = min(obj.img_width, center(1) + width);
bottom = min(obj.img_height, center(2) + height);

frame = [left top right bottom];