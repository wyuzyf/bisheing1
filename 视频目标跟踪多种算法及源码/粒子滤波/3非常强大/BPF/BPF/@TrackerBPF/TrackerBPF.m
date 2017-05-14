function obj = TrackerBPF(varargin)
% PURPOSE : Constructor of @TrackerBPF
% AUTHOR: Kenji Okuma
% DATE: January 2007
% =========================================================================

if nargin == 0
    obj = initFields;
    obj = class(obj, 'TrackerBPF');
elseif isa(varargin{1}, 'TrackerBPF')
    obj = varargin{1};
else
    obj = initFields;
    obj = class(obj, 'TrackerBPF');
    obj = set(obj, varargin);
end

% -----------------------------------
function obj = initFields()

% version number
obj.version = 1.3;

% name of object 
obj.name = '';

% Resolution
obj.img_width = 320;        
obj.img_height = 240;

% detector option
obj.online_detector = 1;    % use online detector by default 

% targets
obj.max_num_targets = 16;   % max number of targets
obj.box_width = 24;         % original width of target (i.e., scale = 1)
obj.box_height = 24;        % original height of target (i.e., scale = 1)
obj.targets = [];           % information of targets

% particles
obj.pc_num = 30;            % number of particles for each player

% color histogram
obj.hsv_lambda = -20;       % lambda for computing likelihood
obj.hsv_alpha = 0.5;        % alpha control weight for hs and v histogram
                            %   e.g., hsv_alpha = .9 for hs:90% and v:10%
obj.hsv_num_box	= 2;		% number of sub-sections (max. 2) for color histogram

% Adaboost confidence
obj.abc_lambda = -0.5;      % labmda for adaboost confindence 

% BPF alpha
obj.BPF_alpha = .5;         % boosting alpha [0,1]: 1 is full boosting
                            %                       0 is no boosting

% prior noise covariance
obj.prior_noise_cov = [5 0 0; 0 5 0; 0 0 1];

% debug
obj.verbosity = 0;          % any value > 0 would start displaying output texts 




