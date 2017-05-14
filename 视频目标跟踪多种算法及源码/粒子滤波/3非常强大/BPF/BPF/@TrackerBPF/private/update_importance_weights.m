function obj = update_importance_weights(obj)
% PURPOSE : Update importance weight of each particle
% INPUT : - obj       = @TrackerBPF object
% OUTPUT : - obj       = updated @TrackerBPF object
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

targets = obj.targets;

i = 1;
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    if targets{i}.status
        % Compute importance weights
        lkhd = targets{i}.pcweight;

        % mixture proposal: adaboost detections + trans_prior
        if targets{i}.num_ada > 0 &&  targets{i}.boosting_status       

            temp_particle = targets{i}.particle;
            temp_boost_mu = targets{i}.boost_mu;
            temp_particle(3, :) = temp_particle(3, :) * obj.box_width; % box_width = box_height
            temp_boost_mu(3, :) = temp_boost_mu(3, :) * obj.box_width; % box_width = box_height 
            
            prop_ada = normpdf(temp_particle, temp_boost_mu, [], targets{i}.boost_cov);
            trans_prior = targets{i}.trans_prior;
            
            proposal = targets{i}.BPF_alpha.*prop_ada + (1 - targets{i}.BPF_alpha).*trans_prior;

            targets{i}.pcweight = lkhd.*trans_prior./proposal;
                
        else  % bootstrap case (i.e., trans_prior = proposal)
            targets{i}.pcweight = lkhd;
        end

        
        % Resampling 
        inIndex = 1:targets{i}.num_pc;
        targets{i}.pcweight = targets{i}.pcweight./sum(targets{i}.pcweight);
        outIndex = deterministicR(inIndex, targets{i}.pcweight(inIndex)');
        tmp_particle = targets{i}.particle;
        tmp_particlePast = targets{i}.particlePast;
        tmp_pc_boosting_status = targets{i}.pc_boosting_status;
        targets{i}.particle = tmp_particle(:, outIndex);
        targets{i}.particlePast = tmp_particlePast(:, outIndex);
        targets{i}.pc_boosting_status = tmp_pc_boosting_status(outIndex);
        targets{i}.pcweight = ones(1,targets{i}.num_pc)/targets{i}.num_pc;
        
        % Update target centers and scale
        targets{i}.center_img = mean(targets{i}.particle(1:2,:), 2);
        targets{i}.scale = mean(targets{i}.particle(3,:), 2); 
        
        if obj.verbosity > 0
            fprintf('target%d: center = (%3.2f, %3.2f) height = width = %2.2f \n', i, ...
                targets{i}.center_img(1), targets{i}.center_img(2), obj.box_height*targets{i}.scale);
        end
    end   
    i = i + 1;
end

if obj.verbosity > 0
    fprintf('\n');
end

obj.targets = targets;
