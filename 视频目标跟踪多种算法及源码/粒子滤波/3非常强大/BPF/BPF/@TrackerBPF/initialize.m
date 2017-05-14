function obj = initialize(obj, boost, filename)
% PURPOSE : Initializing parameters for tracking
% INPUT : - obj = @TrackerBPF object
%         - boost = detection history by adaboost detecton
%         - filename = name of current image
% OUTPUT : - obj = initialized @TrackerBPF object
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

[null numObjs] = size(boost);

% hsv color
hsv_obj = MColorHistogramHSV(filename, 10, 10, 10, 'local');

targets = cell(1, obj.max_num_targets);

for i = 1:numObjs,
    leftTop = boost(1:2,i) + [1;1];
    targets{i}.scale = boost(3, i);

    % Expand the coordinate to the double sized image and then compute the
    % reference color histogram of the target
    width = targets{i}.scale*obj.box_width;
    height = targets{i}.scale*obj.box_height;
    center = leftTop + [width/2; height/2];
    frame = locate_frame(obj, center, round(height), round(width));

    % compute HSV color histogram in the resized image
    [targets{i}.model.color, null, null, null] = compute_HSVhist(hsv_obj, frame, obj.hsv_num_box);

	% the center of the tracking box
    targets{i}.center_img = round(center);
    

	% Initial particles sampled from prior distribution
	targets{i}.x_prior_mu = 0;
    targets{i}.x_prior_cov = [.5 0 0; 0 .5 0; 0 0 eps];
    noise = randnorm(obj.pc_num, [0; 0; 0], [], targets{i}.x_prior_cov);
    noise(3, :) = noise(3, :) / obj.box_width; % box_width = box_height
    tmp = [center; boost(3,i)];
    tmp = repmat(tmp, [1 obj.pc_num]);
    tmp(1:3,:) = tmp(1:3,:) + noise;
    targets{i}.particle = tmp;
    targets{i}.particlePast = targets{i}.particle;
    targets{i}.particleTemp = zeros(2, obj.pc_num);
    targets{i}.pcstatus = ones(1, obj.pc_num);  % 1 if a particle is alive
    targets{i}.pcweight = ones(1, obj.pc_num)/obj.pc_num;
    targets{i}.pc_boosting_status = ones(1, obj.pc_num); % if pc is boosted, then 1. Otherwise 0.
    
    targets{i}.num_ada = obj.pc_num*obj.BPF_alpha; 
    targets{i}.num_pc = obj.pc_num;    
    targets{i}.max_num_targets = obj.max_num_targets;
    targets{i}.interaction = zeros(1, obj.max_num_targets); % no interaction by default
	targets{i}.status = true;  % true if the target is active
	targets{i}.conf = 0.0;     % confidence of the target
    
    targets{i}.prev_pc_boosting_status = ones(1, obj.pc_num); 
        
    targets{i}.boost_mu = repmat([center; boost(3,i)], [1 targets{i}.num_pc]);
    targets{i}.boost_cov = [1 0 0; 0 1 0; 0 0 1];    
    targets{i}.trans_prior = zeros(1, obj.pc_num);
    
    targets{i}.boosting_status = false;  % true if the target is boosted. false otherwise.
    targets{i}.BPF_alpha = ones(1, obj.pc_num)*obj.BPF_alpha;
   
end

obj.targets = targets;