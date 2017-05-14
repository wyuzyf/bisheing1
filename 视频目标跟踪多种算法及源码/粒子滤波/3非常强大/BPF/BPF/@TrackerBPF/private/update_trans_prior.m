function obj = update_trans_prior(obj, boost, NNboost, hsv_obj)
% PURPOSE : Update transition prior for each target
% INPUT : - obj          = @TrackerBPF object
%         - boost        = detection information
%         - NNboost      = neighboring detection
%         - hsv_obj      = hsv color hist obj
% OUTPUT : - obj         = updated@TrackerBPF object
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

targets = obj.targets;

% Mixture of boosting and particle filtering
i = 1;
maxDist = 1e003;

while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    % nonBPF igonores the following case
    if targets{i}.status && obj.BPF_alpha > 0
        % boosted detection is overlapped with a target
        detection_present = false;
        if NNboost(1,i) < maxDist
            % location and scale of boosted detection
            detection = boost(1:3,NNboost(2,i));
            detection_present = true;
            % update model
            height = obj.box_height*detection(3);
            width = obj.box_width*detection(3);
            % move left-top corner to center
            detection(1) = detection(1) + width*0.5;
            detection(2) = detection(2) + height*0.5;
            center = detection(1:2);
            frame = round(locate_frame(obj, center, height, width));
            targets{i} = update_model(obj, targets{i}, frame, hsv_obj);
        end
        if  detection_present
            boost_cov = [1 0 0; 0 1 0; 0 0 1];
            % Dynamics option: Simply gaussian noise
            noise = randnorm(targets{i}.num_pc, [0; 0; 0], [], boost_cov);
            noise(3, :) = noise(3, :) / obj.box_width; % box_width = box_height

            randIndex = randperm(targets{i}.num_pc);
            num_ada = round(obj.BPF_alpha*obj.pc_num);
            % update current particles and their boosted status
            tmp_particle = targets{i}.particle;
            targets{i}.particle(:, 1:num_ada) = repmat(detection, [1 num_ada]) + noise(:,1:num_ada);
            targets{i}.particle(:, (num_ada+1):end) = tmp_particle(:, randIndex((num_ada+1):end));

            % update previous particles
            tmp_particlePast = targets{i}.particlePast;
            targets{i}.particlePast(:, 1:num_ada) = repmat(detection, [1 num_ada]);
            targets{i}.particlePast(:, (num_ada+1):end) = tmp_particlePast(:, randIndex((num_ada+1):end));

            % update boosted status of each particle
            targets{i}.prev_pc_boosting_status =  targets{i}.pc_boosting_status;
            targets{i}.pc_boosting_status(1:num_ada) = 1;
            targets{i}.pc_boosting_status((num_ada+1):end) = 0;

            targets{i}.num_ada = num_ada;

            % update mixture transition prior
            tmp_trans_prior = normpdf(targets{i}.particle,  targets{i}.x_prior_mu, [], targets{i}.x_prior_cov);
            targets{i}.trans_prior = tmp_trans_prior;

            % update boosted status of each particle
            targets{i}.boost_mu = repmat(detection, [1 targets{i}.num_pc]);
            targets{i}.boost_cov = boost_cov;

            targets{i}.boosting_status = true;
        else
            % update mixture transition prior
            tmp_trans_prior = normpdf(targets{i}.particle,  targets{i}.x_prior_mu, [], targets{i}.x_prior_cov);
            targets{i}.trans_prior = tmp_trans_prior;
            % no particles boosted
            targets{i}.prev_pc_boosting_status =  targets{i}.pc_boosting_status;
            targets{i}.pc_boosting_status = targets{i}.pc_boosting_status*0;
            % no detection nearby
            targets{i}.boosting_status = false;
            % update alpha
            targets{i}.BPF_alpha = targets{i}.BPF_alpha*0;
        end
    end
    i = i + 1;
end

obj.targets = targets;

