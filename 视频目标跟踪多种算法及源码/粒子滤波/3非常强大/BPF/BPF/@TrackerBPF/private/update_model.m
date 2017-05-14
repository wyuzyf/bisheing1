function target = update_model(obj, target, new_frame, hsv_obj)
% PURPOSE : Update the model(hsv color histogram)
% INPUT : - obj         = @TrackerBPF object
%         - target      = target's information (a cell)
%         - new_frame   = image where a new model must be based on
%         - hsv_obj     = hsv color hist obj for computing hsv
% OUTPUT : - target      = updated target (a cell)
% Authors: Kenji Okuma
% Date: January 2007
% =========================================================================

% update HSV color histogram model
[updatedTarget, up, low, bound] = compute_HSVhist(hsv_obj, new_frame, obj.hsv_num_box);
for j = 1:obj.hsv_num_box,
      target.model.color.HS(:,:,j) = updatedTarget.HS(:,:,j);
      target.model.color.V(:,j) = updatedTarget.V(:,j);
end
