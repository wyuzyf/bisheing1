function obj = merge_targets(obj)
% PURPOSE : Merging components
% INPUT : - obj  = @TrackerBPF object
% OUTPUT : - obj  = updated @TrackerBPF object
% Authors: Kenji Okuma
% Date: January 2007
% =========================================================================

targets = obj.targets;

numActiveTargets = 0;
i = 1;
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
	if targets{i}.status
		numActiveTargets = numActiveTargets + 1;
	end
	i = i + 1;
end		

i = 1;	
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    x = targets{i}.center_img(1) - 0.5*targets{i}.scale*obj.box_width*.5;
    y = targets{i}.center_img(2) - 0.5*targets{i}.scale*obj.box_height*.75;
    A = [x y targets{i}.scale*obj.box_width*.5 targets{i}.scale*obj.box_height*.75];

    % association to other existing targets
    j = 1;
    while ~isemptycell(targets(j)) && j <= obj.max_num_targets
        if targets{i}.status && targets{j}.status && j ~= i,
            xx = targets{j}.center_img(1) - 0.5*targets{j}.scale*obj.box_width*.5;
            yy = targets{j}.center_img(2) - 0.5*targets{j}.scale*obj.box_height*.75;
            B = [xx yy targets{j}.scale*obj.box_width*.5 targets{j}.scale*obj.box_height*.75];
            [overlap normOverlap] = rectintC(A,B);
            % significant overlap with other target
            sizeRatio = B(3)/A(3);
            % decide which target to remove based on adaboost confidence
            if targets{i}.conf > 0 && targets{j}.conf > 0                               
                if targets{i}.conf > targets{j}.conf
                    removedObj = j;
                    mergedObj = i;
                else
                    removedObj = i;
                    mergedObj = j;
                end
            else % either target i or j (or maybe both) is just added or initialized
                if targets{i}.conf == 0
                    removedObj = j;
                    mergedObj = i;                    
                end
                if targets{j}.conf == 0
                    removedObj = i;
                    mergedObj = j;
                end
            end
            
            if normOverlap > .80 
                targets{removedObj}.interaction = zeros(1, obj.max_num_targets); % all off
                targets{removedObj}.pc_boosting_status = zeros(1, obj.pc_num); % if pc is boosted, then 1. Otherwise 0.
                targets{removedObj}.status = false;

                % update interaction status 
                k = 1;
                while ~isemptycell(targets(k)) && k <= obj.max_num_targets
                    if targets{k}.status                      
                        targets{k}.interaction(removedObj) = false;
                    end
                    k = k + 1;
                end
                
				numActiveTargets = numActiveTargets - 1;
				% update other mixture weights
				k = 1;
				while ~isemptycell(targets(k)) && k <= obj.max_num_targets
					if targets{k}.status
						targets{k}.mixture_weight = 1/numActiveTargets;
					end
					k = k + 1;
				end				
				
            elseif normOverlap > 0
                targets{i}.interaction(j) = true;
                targets{j}.interaction(i) = true;
            elseif normOverlap == 0
                targets{i}.interaction(j) = false;
                targets{j}.interaction(i) = false;
            end
        end
        j = j + 1;
    end
    i = i + 1;
end

obj.targets = targets;


