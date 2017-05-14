function drawDetections(obj, boost, colortable)
% PURPOSE : Draw a detection box for each target
% INPUT : - obj = @TrackerBPF object
%         - boost = detection history by adaboost detecton
%         - colortable = color table for drawing
% OUTPUT : 
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

lineWidth = 2;

black = colortable(17,:);
[null, num_boost] = size(boost);
for i = 1:num_boost,
  x = boost(1,i);
  y = boost(2,i);
  scale = boost(3,i);
  width = scale*obj.box_width;
  height = scale*obj.box_height;
      rectangle('Position', [round(x), round(y), round(width), round(height)], ...
          'EdgeColor', 'black', 'lineWidth', lineWidth);
end