function hsv_img = drawConfMap(obj, filename)
% PURPOSE : Draw confidence map based on hsv color histogram
% INPUT : - obj = @TrackerBPF object
%         - filename = name of current image file
% OUTPUT : - hsv_img     = confidence map for hsv
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

targets = obj.targets;

hsv_img = zeros(obj.img_height, obj.img_width);

% compute features
hsv_obj = MColorHistogramHSV(filename, 10, 10, 10, 'local');
% ========================

i = 1;
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    if targets{i}.status
        target = targets{i};
        width = round(target.scale*obj.box_width);
        height = round(target.scale*obj.box_height);
        leftTop = max(round([width*.5; height*.5] + 1), round(target.center_img - [width*.7; height*.7]));
        rightBottom = min(round([obj.img_width - width*.5; obj.img_height - height*.5]), round(target.center_img + [width*.7; height*.7]));
        mapWidth = length(leftTop(1):rightBottom(1));
        mapHeight = length(leftTop(2):rightBottom(2));

        rect_indx = 1;
        numPC = mapWidth*mapHeight;

        rect_hsv = zeros(numPC, 4, obj.hsv_num_box);
        for x = leftTop(1):rightBottom(1)
            for y = leftTop(2):rightBottom(2)
                center = [x; y];
                % Normalize the pixels in the frame
                frame = locate_frame(obj, center, height, width);

                % record detection box

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
                    rect_hsv(rect_indx,:) = [left top right bottom];
                elseif obj.hsv_num_box == 2
                    divider = round((bottom - top)/2 + top);
                    rect_hsv(rect_indx,:,1) = [left top right divider];
                    rect_hsv(rect_indx,:,2) = [left divider right bottom];
                end
                rect_hsv = round(rect_hsv);
                rect_indx = rect_indx + 1;
            end
        end


        if obj.hsv_num_box == 1
            [HShists Vhists] = gethistogram(hsv_obj, rect_hsv, 1);
        elseif obj.hsv_num_box == 2
            [HShists_up Vhists_up] = gethistogram(hsv_obj, rect_hsv(:,:,1), 1);
            [HShists_low Vhists_low] = gethistogram(hsv_obj, rect_hsv(:,:,2), 1);
            HShists = zeros(numPC, 100, obj.hsv_num_box);
            Vhists = zeros(numPC, 10, obj.hsv_num_box);
            HShists(:,:,1) = HShists_up;
            HShists(:,:,2) = HShists_low;
            Vhists(:,:,1) = Vhists_up;
            Vhists(:,:,2) = Vhists_low;
        end
        hsv_weights = zeros(obj.hsv_num_box, numPC);
        % hsv color weights
        for n = 1:obj.hsv_num_box
            hs_model = reshape(target.model.color.HS(:,:,n), [1 100]);
            v_model = target.model.color.V(:,n)';
            target_hs = repmat(hs_model, [numPC 1]);
            target_v =  repmat(v_model, [numPC 1]);
            hs_weights = compute_bhat(HShists(:,:,n)', target_hs');
            hsv_weights(n,:) = hs_weights + compute_bhat(Vhists(:,:,n)', target_v');
        end
        if obj.hsv_num_box > 1
            hsv_weights = prod(hsv_weights(1:2,:));
        end
        % normalize only for the drawing purpose
        hsv_weights = hsv_weights - min(hsv_weights);
        hsv_weights = exp(obj.hsv_lambda*hsv_weights);
        hsv_weights = hsv_weights/max(hsv_weights);

        count = 1;
        for x = leftTop(1):rightBottom(1)
            for y = leftTop(2):rightBottom(2)
                hsv_img(y, x) = hsv_weights(count);
                count = count + 1;
            end
        end
    end

    i = i + 1;
end

