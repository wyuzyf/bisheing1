function val = get(obj, propName)
% PURPOSE : Get properties of @TrackerBPF object
% 			e.g. val = get(obj, 'propName')
% INPUT: - obj = @TrackerBPF object
% OUTPUT: - val = value of the specific property of @TrackerBPF
% AUTHOR: Kenji Okuma
% DATE: Jan 2007
% =========================================================================

switch propName
    case 'version'
        val = obj.version;
    case 'name'
        val = obj.name;
    case 'img_width'
        val = obj.img_width;
    case 'img_height'
        val = obj.img_height;
    case 'online_detector'
        val = obj.online_detector;			
    case 'max_num_targets'
        val = obj.max_num_targets;
    case 'box_width'
        val = obj.box_width;
    case 'box_height'
        val = obj.box_height;
    case 'targets'
        val = obj.targets;
    case 'pc_num'
        val = obj.pc_num;
    case 'hsv_lambda'
        val = obj.hsv_lambda;
    case 'hsv_alpha'
        val = obj.hsv_alpha;
    case 'hsv_num_box'
        val = obj.hsv_num_box;
    case 'abc_lambda'
        val = obj.abc_lambda;
    case 'BPF_alpha'
        val = obj.BPF_alpha;
    case 'prior_noise_cov'
        val = obj.prior_noise_cov;
    case 'verbosity'
        val = obj.verbosity;
    otherwise
        error([propName,' Is not a valid @TrackerBPF property']);
end

