function obj = set(obj, varargin)
% PURPOSE : Set properties of @TrackerBPF object
% 			obj = set(obj, 'propName', val, ...)
% AUTHOR: Kenji Okuma
% DATE: Jan 2007
% =========================================================================

if iscell(varargin{1})
    args = varargin{1};
else
    args = varargin;
end

while length(args) >= 2,
    prop = args{1};
    val = args{2};
    args = args(3:end);
    switch prop
        case 'version'
            obj.version = val;
        case 'name'
            obj.name = val;
        case 'img_width' 
            obj.img_width = val;
        case 'img_height'
            obj.img_height = val;
        case 'online_detector'
            obj.online_detector = val;			
        case 'max_num_targets'
            obj.max_num_targets = val;
        case 'box_width'
            obj.box_width = val;
        case 'box_height'
            obj.box_height = val;
        case 'targets'
            obj.targets = val;
        case 'pc_num' 
            obj.pc_num = val;
        case 'hsv_lambda' 
            obj.hsv_lambda = val;        
        case 'hsv_alpha'
            obj.hsv_alpha = val;
        case 'hsv_num_box'
            obj.hsv_num_box = val;
        case 'abc_lambda'
            obj.abc_lambda = val;
        case 'BPF_alpha'
            obj.BPF_alpha = val;
        case 'prior_noise_cov'
            obj.prior_noise_cov = val;
        case 'verbosity'
            obj.verbosity = val;            
        otherwise
            warning('TrackerBPF:unknownProperties', 'Unknown properties.');
    end
end

