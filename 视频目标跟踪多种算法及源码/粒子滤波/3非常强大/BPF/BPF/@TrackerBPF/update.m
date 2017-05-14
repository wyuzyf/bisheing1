function obj = update(obj, filename, gray_img, boost, iImage, iSquareImage)
% PURPOSE : Update tracking information for all targets
% INPUT : - obj          = @TrackerBPF object
%         - filename     = name of the current image
%         - gray_img     = gray image of cur_img for adaboost confidence
%         - boost        = history of adaboost detection results
%         - iImage       = integral image of gray_img
%         - iSquareImage = squared integral image of gray_img
% OUTPUT: - obj          = updated @TrackerBPF object
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

% ========================
% compute features
hsv_obj = MColorHistogramHSV(filename, 10, 10, 10, 'local');
% ========================

% propagate particles
obj = propagate_particles(obj);

% add targets
obj = add_targets(obj, boost, hsv_obj);

% update observation likelihoods
obj = update_obsv_lkhds(obj, hsv_obj);

% update importance weights
obj = update_importance_weights(obj);

% remove targets 
obj = remove_targets(obj, gray_img, hsv_obj, iImage, iSquareImage);

% merge targets
obj = merge_targets(obj);

