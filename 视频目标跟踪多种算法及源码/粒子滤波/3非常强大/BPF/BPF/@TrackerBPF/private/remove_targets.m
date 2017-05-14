function obj = remove_targets(obj, gray_img, hsv_obj, iImage, iSquareImage)
% PURPOSE : Remove targets based on the status and dist. of particles
% INPUT :  - obj          = @TrackerBPF object
%          - gray_img     = gray image for adaboost confidence map
%          - hsv_obj      = hsv color hist obj
%          - iImage       = integral image of gray_img
%          - iSquareImage = squared integral image of gray_img
% OUTPUT : - obj          = updated @TrackerBPF object
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

targets = obj.targets;

numActiveTargets = 0;
i = 1;
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    if targets{i}.status
        numActiveTargets = numActiveTargets + 1;
        x = targets{i}.center_img(1);
        y = targets{i}.center_img(2);
        scale = targets{i}.scale;
        width = scale*obj.box_width*.5;
        height = scale*obj.box_height*.75;
        leftTopX = round(x - 0.5*width);
        leftTopY = round(y - 0.5*height);

        num_pc = targets{i}.num_pc;
        count = sum(targets{i}.pcstatus);

        % compute overlap between a box and an entire image
        A = [1 1 obj.img_width obj.img_height];
        B = [leftTopX leftTopY width height];
        [overlap normOverlap] = rectintC(A,B);

		% remove target if more than 10% of its particles or it is outside of the image
        if count < num_pc*0.9 || normOverlap < 1
            if obj.verbosity > 0
                disp('Remove one target');
            end
            numActiveTargets = numActiveTargets - 1;
            targets{i}.interaction = zeros(1, obj.max_num_targets); % all off
            targets{i}.pc_boosting_status = zeros(1, obj.pc_num); % if pc is boosted, then 1. Otherwise 0.
            targets{i}.status = false;
        end
    end
    i = i + 1;
end

%fprintf('\n');

% The following works only if you use online detector and its adaboost confidence provided in mex
if numActiveTargets > 0 && obj.online_detector
    % confs = [ hsv_weight_1 abc_weight_1 ]
    %         [    ...          ...       ]
    %         [ hsv_weight_i abc_weight_i ]
    confs = compute_conf(obj, gray_img, hsv_obj, iImage, iSquareImage);
    confs = confs';

    % re-scale confs
    confs(1,:) = confs(1,:) - min(confs(1,:));  % re-scale hsv similarity
    confs(2,:) = confs(2,:) - max(confs(2,:));  % re-scale adaboost confidence

    % compute the probability of boosting conf
    % confs(1,:) = exp(-confs(1,:));
    confs(2,:) = exp(1e-003*confs(2,:));
    conf = confs(2,:);
    thresh = 0.1;

    i = 1;
    index = 1;
    while ~isemptycell(targets(i)) && i <= obj.max_num_targets
        if targets{i}.status
            targets{i}.conf = conf(index);
            removeTarget = false;
            % BPF case			
            if obj.BPF_alpha > 0
                if conf(index) < thresh % mixture weight cannot be used at the moment
                    removeTarget = true;
                end
            else % non BPF case
                if conf(index) < thresh
                    removeTarget = true;
                end
            end

			% remove target based on adaboost confidence
            if removeTarget
                if obj.verbosity > 0
                    disp('Remove one target!!!!!');
                end
                numActiveTargets = numActiveTargets - 1;
                targets{i}.interaction = zeros(1, obj.max_num_targets); % all off
                targets{i}.pc_boosting_status = zeros(1, obj.pc_num); % if pc is boosted, then 1. Otherwise 0.
                targets{i}.status = false;

                % update interaction status
                k = 1;
                while ~isemptycell(targets(k)) && k <= obj.max_num_targets
                    if targets{k}.status
                        targets{k}.interaction(i) = false;
                    end
                    k = k + 1;
                end
            end
            index = index + 1;
        end
        i = i + 1;
    end
end

obj.targets = targets;