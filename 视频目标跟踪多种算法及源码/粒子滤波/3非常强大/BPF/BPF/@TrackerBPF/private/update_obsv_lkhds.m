function obj = update_obsv_lkhds(obj, hsv_obj)
% PURPOSE : Compute observation likelihoods
% INPUT:   - obj          = @TrackerBPF object
%          - hsv_obj      = hsv color hist obj
% OUTPUT : - obj          = updated @TrackerBPF object
% AUTHOR : Kenji Okuma
% DATE : January 2007
% =========================================================================

targets = obj.targets;

i = 1;
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    if targets{i}.status
        rect_indx = 1;
        pcIndx = find(targets{i}.pcstatus);
        numPC = length(pcIndx);
        rect_hsv = zeros(numPC, 4, obj.hsv_num_box);
        for j = pcIndx,
            center = round(targets{i}.particle(1:2,j));
            height = obj.box_height*targets{i}.particle(3,j);
            width = obj.box_width*targets{i}.particle(3,j);

            % Normalize the pixels in the frame
            frame = locate_frame(obj, center, height, width);

            % boxes
            left = frame(1);
            top = frame(2);
            right = frame(3);
            bottom = frame(4);
            rectWidth = right - left;
            rectHeight = bottom - top;

			% color region is rectangular box inside of square box
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

            rect_indx = rect_indx + 1;
        end

        hsv_weights = ones(1, numPC)/numPC;

        % hsv color weights
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
        for j = 1:obj.hsv_num_box
            hs_model = reshape(targets{i}.model.color.HS(:,:,j), [1 100]);
            v_model = targets{i}.model.color.V(:,j)';
            target_hs = repmat(hs_model, [numPC 1]);
            target_v =  repmat(v_model, [numPC 1]);
            hs_weights = compute_bhat(HShists(:,:,j)', target_hs');
            hsv_weights(j,:) = hs_weights*obj.hsv_alpha + compute_bhat(Vhists(:,:,j)', target_v')*(1-obj.hsv_alpha);
        end


        if obj.hsv_num_box > 1
            for j = 1:2
                hsv_weights(j,:) = hsv_weights(j,:) - min(hsv_weights(j,:));
                hsv_weights(j,:) = exp(obj.hsv_lambda*hsv_weights(j,:));
                hsv_weights(j,:) = hsv_weights(j,:)/sum(hsv_weights(j,:));
            end

            w = hsv_weights(1,:) .* hsv_weights(2,:);
			hsv_weights = w/sum(w);
			
        else
            % normalize weights
            hsv_weights = hsv_weights - min(hsv_weights);
            hsv_weights = exp(obj.hsv_lambda*hsv_weights);
            hsv_weights = hsv_weights/sum(hsv_weights);
        end
        
        targets{i}.pcweight(1, pcIndx) = hsv_weights;
    end
    
    % Check if the particles are dead (i.e., out of image) or not
    targets{i}.pcstatus = checkstatus(obj, targets{i}.particle);
    
    i = i + 1;
end


obj.targets = targets;





