function obj = add_targets(obj, boost, hsv_obj)
% PURPOSE : Adding targets 
% INPUT : - obj          = @TrackerBPF object
%         - boost        = Adaboost detection results (a cell)
%         - hsv_obj      = hsv color hist obj for computing hsv
% OUTPUT : - obj         = updated @TrackerBPF object
% Authors: Kenji Okuma
% Date: January 2007
% =========================================================================

targets = obj.targets;

[null, num_boost] = size(boost);
new_target = -1*ones(1,num_boost);

maxDist = 1e003;
NNboost = maxDist*ones(2, obj.max_num_targets);

count = 0;
numActiveTargets = 0;
numNewTargets = 0; 

if num_boost > 0
    for i = 1:num_boost,
        x = boost(1,i);
        y = boost(2,i);
        x_ = x + boost(3,i)*obj.box_width;
        y_ = y + boost(3,i)*obj.box_height;
        A = [x y boost(3,i)*obj.box_width boost(3,i)*obj.box_height];

        % overlaps between boosting detections and existing targets
        j = 1;
        while ~isemptycell(targets(j)) && j <= obj.max_num_targets
            if targets{j}.status
                xx = targets{j}.center_img(1) - 0.5*targets{j}.scale*obj.box_width;
                yy = targets{j}.center_img(2) - 0.5*targets{j}.scale*obj.box_height;
                xx_ = xx + targets{j}.scale*obj.box_width;
                yy_ = yy + targets{j}.scale*obj.box_height;
                B = [xx yy targets{j}.scale*obj.box_width targets{j}.scale*obj.box_height];
                [overlap normOverlap]= rectintC(A,B);                
                % some overlap
                if normOverlap ~= 0
                    % compute distance of the center of two boxes
                    dist = sqrt(((x + x_)/2 - (xx_ + xx)/2)^2 + ((y + y_)/2 - (yy_ + yy)/2)^2);
                    
                    % Remember the boost detection with minimum distance and
                    % significant overlap
                    if normOverlap > 0.4 && dist < NNboost(1,j),
                        NNboost(1,j) = dist;
                        NNboost(2,j) = i;
                    end
                end
            end
            j = j + 1;
        end
        
        % Check if the boosting detection has any overlap with existing
        % particles
        count = 0;
        numActiveTargets = 0;
        j = 1;
        while ~isemptycell(targets(j)) && j <= obj.max_num_targets
            if targets{j}.status
                numActiveTargets = numActiveTargets + 1;
                for k=1:obj.pc_num,
                    px = targets{j}.particle(1,k) - 0.5*targets{j}.particle(3,k)*obj.box_width;
                    py = targets{j}.particle(2,k) - 0.5*targets{j}.particle(3,k)*obj.box_height;
                    B = [px py targets{j}.particle(3,k)*obj.box_width ...
                        targets{j}.particle(3,k)*obj.box_height];
                    [overlap normOverlap] = rectintC(A,B);
                    if normOverlap < 0.3
                        count = count + 1;
                    end
                end
            end
            j = j + 1;
        end
        % If no overlap with all particles of all targets, then create new
        % target
        if count == obj.pc_num*numActiveTargets,
            numNewTargets = numNewTargets + 1;
            new_target(numNewTargets) = i;
        end
    end
    
    % update transition prior	
    obj = update_trans_prior(obj, boost, NNboost, hsv_obj);
	  targets = obj.targets;
	
    % Create new target
    if numNewTargets > 0 && obj.verbosity > 0
        tmp = sprintf('# of new targets: %d', numNewTargets); 
        disp(tmp);
    end

    % Insert new targets
    for i = 1:min(numNewTargets, obj.max_num_targets - numActiveTargets)
        % Find the position to insert the new target
        insert = -1;
        j = 1;
        while ~isemptycell(targets(j)) && j <= obj.max_num_targets && insert < 0 
            if ~targets{j}.status % dead
                insert = j;
            end
            j = j + 1;
        end
        j = j - 1;
        
        % Create new target with new index
        if ~(insert < 0 && j == obj.max_num_targets)
						numActiveTargets = numActiveTargets + 1;
            if insert < 0 && j < obj.max_num_targets
                insert = j + 1;
            end
            % =====================================================================
            % Insert the new target
            leftTop = boost(1:2, new_target(i));
            targets{insert}.scale = boost(3, new_target(i));
            % Expand the coordinate to the double sized image and then compute the
            % reference color histogram of the target
            width = targets{insert}.scale*obj.box_width;
            height = targets{insert}.scale*obj.box_height;
            center = leftTop + [width/2; height/2];
            frame = locate_frame(obj, round(center), round(height), round(width));

            % HSV color histogram
            % hsv_obj = MColorHistogramHSV(filename, 10, 10, 10, 'local');
            [targets{insert}.model.color, null, null, null] = compute_HSVhist(hsv_obj, frame, obj.hsv_num_box);

			
			targets{insert}.center_img = round(center);
			
            noise = randnorm(obj.pc_num, [0; 0; 0], [], [1 0 0; 0 1 0; 0 0 1]);
            noise(3, :) = noise(3, :) / obj.box_width; % box_width = box_height

            
            tmp = [center; boost(3, new_target(i))];
			targets{insert}.x_prior_mu = 0;
            targets{insert}.x_prior_cov = [.5,0 0;0 .5 0;0 0 eps];
            targets{insert}.particle = repmat(tmp, [1 obj.pc_num]) + noise;
            targets{insert}.particlePast = targets{insert}.particle - noise;
            targets{insert}.pcstatus = ones(1,obj.pc_num);
            targets{insert}.pcweight = ones(1,obj.pc_num)/obj.pc_num;
			
            targets{insert}.num_ada = obj.pc_num*obj.BPF_alpha;
            targets{insert}.num_pc = obj.pc_num;

            targets{insert}.status = true;
            targets{insert}.interaction = zeros(1,obj.max_num_targets); % no interaction by default
            targets{insert}.pc_boosting_status = ones(1, obj.pc_num); % if pc is boosted, then 1. Otherwise 0.
            targets{insert}.prev_pc_boosting_status = ones(1, obj.pc_num);
            targets{insert}.conf = 0.0;
			
            targets{insert}.boost_mu = repmat([center; boost(3, new_target(i))], [1 targets{insert}.num_pc]);
            targets{insert}.boost_cov = [1 0 0; 0 1 0; 0 0 1];
            targets{insert}.trans_prior = zeros(1, obj.pc_num);
            
            targets{insert}.boosting_status = false;
            targets{insert}.BPF_alpha = ones(1, obj.pc_num)*obj.BPF_alpha;            

        end
    end
end

obj.targets = targets;