function confs = compute_conf(obj, gray_img, hsv_obj, iImage, iSquareImage)
% PURPOSE : compute confidence of the target
% INPUT:   - obj          = @TrackerBPF object
%          - gray_img     = gray image for adaboost confidence map
%          - hsv_obj      = hsv color hist obj
%          - iImage       = integral image of gray_img
%          - iSquareImage = squared integral image of gray_img
% OUTPUT : - confs        = confidence value for each target
% AUTHOR : Kenji Okuma
% DATE : January 2007
% =========================================================================

targets = obj.targets;
numActiveTargets = 0;
i = 1;
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    if targets{i}.status
        numActiveTargets = numActiveTargets + 1;
        targetIndex(numActiveTargets) = i;
        center = targets{i}.center_img;
        height = obj.box_height*targets{i}.scale;
        width = obj.box_width*targets{i}.scale;

        % Normalize the pixels in the frame
        frame = locate_frame(obj, center, height, width);

        % record detection box
        frame = round(frame);
        rect_abc(numActiveTargets, :) = frame;
        left = frame(1);
        top = frame(2);
        right = frame(3);
        bottom = frame(4);
        rectWidth = right - left;
        rectHeight = bottom - top;

        widthOffset = round(rectWidth/4.0);
        heightOffset = round(rectHeight/8.0);
        left = left + widthOffset;
        top = top + heightOffset;
        right = right - widthOffset;
        bottom = bottom - heightOffset;
        if obj.hsv_num_box == 1
            rect_hsv(numActiveTargets, :) = round([left top right bottom]);
        elseif obj.hsv_num_box == 2
            divider = round((bottom - top)/2 + top);
            rect_hsv(numActiveTargets, :, 1) = [left top right divider]';
            rect_hsv(numActiveTargets, :, 2) = [left divider right bottom]';
        end
    end
    i = i + 1;
end


hsv_weights = ones(numActiveTargets, 1)*-1;

% adaboost confidence
boxes = zeros(numActiveTargets, 3);
for i = 1:numActiveTargets
    boxes(i, :) = [rect_abc(i,1) rect_abc(i,2) ...
        min((rect_abc(i,3) - rect_abc(i,1))/obj.box_width, (rect_abc(i,4) - rect_abc(i,2))/obj.box_height)];
end
abc_weights = computeAdaBoostConf(gray_img, iImage, iSquareImage, boxes, 0);
abc_weights = abc_weights';

% hsv color weights
if obj.hsv_num_box == 1
    [HShists Vhists] = gethistogram(hsv_obj, rect_hsv, 1);
elseif obj.hsv_num_box == 2
    [HShists_up Vhists_up] = gethistogram(hsv_obj, rect_hsv(:,:,1), 1);
    [HShists_low Vhists_low] = gethistogram(hsv_obj, rect_hsv(:,:,2), 1);
    HShists(:,:,1) = HShists_up;
    HShists(:,:,2) = HShists_low;
    Vhists(:,:,1) = Vhists_up;
    Vhists(:,:,2) = Vhists_low;
end
for i = 1:numActiveTargets
    for j = 1:obj.hsv_num_box
        hs_model = reshape(targets{targetIndex(i)}.model.color.HS(:,:,j), [1 100]);
        v_model = targets{targetIndex(i)}.model.color.V(:,j)';
        target_hs = hs_model;
        target_v =  v_model;
        hs_weights(i,j) = compute_bhat(HShists(i,:,j)', target_hs');
        hsv_weights(i,j) = hs_weights(i)*obj.hsv_alpha + compute_bhat(Vhists(i,:,j)', target_v')*(1-obj.hsv_alpha);
    end
    if obj.hsv_num_box > 1
        tmp_hsv_weights(i,1) = hsv_weights(i,1) * hsv_weights(i,2);
    end
end


hsv_weights = tmp_hsv_weights;

confs = [hsv_weights abc_weights];




