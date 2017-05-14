function drawTargets(obj, colortable, fontColor, lineWidth)
% PURPOSE : Draw the target box(es)
% INPUT : - obj = @TrackerBPF object
%         - colortable = color table to use for drawing
%         - fontColor  = color of the font (optional)
%         - lineWidth  = width of the line (optional)
% OUTPUT : 
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

targets = obj.targets;

if nargin == 3
    lineWidth = 2;
    fontColor = 'k';
end

i = 1;
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    if targets{i}.status
        x = targets{i}.center_img(1);
        y = targets{i}.center_img(2);
        scale = targets{i}.scale;
        width = scale*obj.box_width;
        height = scale*obj.box_height;
        x = (x - 0.5*width);
        y = (y - 0.5*height);

        %text(x+0.4*width, y-.125*obj.box_height, sprintf('%d', i), 'Color', fontColor, 'BackgroundColor', 'w');
        if sum(targets{i}.interaction) == false
            colorIndx = i;
        else
            colorIndx = 17;
        end
        % draw box for hsv color histogram
        rectangle('Position', [round(x), round(y), round(width), round(height)], ...
            'EdgeColor', colortable(colorIndx,:)/255, 'LineWidth', lineWidth);
        rectangle('Position', [round(x+width/4.0), round(y+height/8.0), round(width*.5), round(height*.75)], ...
            'EdgeColor', [0 0 0], 'LineWidth', 1);        
    end
    i = i + 1;
end
