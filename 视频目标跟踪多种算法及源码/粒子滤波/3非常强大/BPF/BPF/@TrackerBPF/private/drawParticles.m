function drawParticles(obj, colortable)
% PURPOSE : Draw particles for each target
% INPUT : - targets = targets' information
%         - colortable = color table for drawing
% OUTPUT : 
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

lineWidth = 2;
targets = obj.targets;
i = 1;
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    if targets{i}.status
        for j = 1:length(targets{i}.particle),
            x = targets{i}.particle(1,j);
            y = targets{i}.particle(2,j);
            scale = targets{i}.particle(3,j);
            width = scale*obj.box_width;
            height = scale*obj.box_height;
            x = x - 0.5*width;
            y = y - 0.5*height;

            rectangle('Position', [round(x), round(y), round(width), round(height)], ...
                'EdgeColor', colortable(i,:)/255, 'lineWidth', lineWidth);
        end
    end
    i = i + 1;
end