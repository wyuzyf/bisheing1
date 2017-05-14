function obj = set(obj, varargin)
% SET           Set properties of the @MColorHistogramHSV object
%   set(obj) shows properties that are allowed to set.
%
%   obj = set(obj, 'propName', val, ...) sets the properties of the 
%   @MColorHistogramHSV object. Possible 'propName' includes:
%     'image'   the image of @MImage format
%     'hbins'   number of bins in H channel
%     'sbins'   number of bins in S channel
%     'vbins'   number of bins in V channel
%     ====================================================================
%     'type'    type of computation
%           'local'     Using a local integral histogram. Suitable for the 
%                       task where there are small number of overlapping 
%                       windows in a small region.
%           'global'    Using a global integral histogram. Suitable for the
%                       task where there are thousands of overlapping
%                       windows distributed throughout the entire image.
%           'buitin'    Using the Matlab builtin function. Suitable for the
%                       task where there are fewer than 500 windows.
%
%   Example:
%     obj = MColorHistogramHSV('moon.tif');
%     obj = set(obj, 'nbins', 15);
%
%   See also MCOLORHISTOGRAMHSV, MIMAGE, GET

% Copyright 2006 Wei-Lwun Lu
% set.m version 1.0

if nargin == 1
    strlist = {'image', 'hbins', 'sbins', 'vbins', 'type'};
    displayProperties(strlist);
else
    propertyArgIn = varargin;
    while length(propertyArgIn) >= 2,
        prop = propertyArgIn{1};
        val = propertyArgIn{2};
        propertyArgIn = propertyArgIn(3:end);
        switch prop
            % image info
            case 'image'
                obj = setimage(obj, val);
                obj = computeCache(obj);

            case 'hbins'
                obj.hbins = val;
                obj.hsbins = obj.hbins * obj.sbins;
                obj = computeCache(obj);
                
            case 'sbins'
                obj.sbins = val;
                obj.hsbins = obj.hbins * obj.sbins;
                obj = computeCache(obj);
            
            case 'vbins'
                obj.vbins = val;
                obj = computeCache(obj);
                
            case 'type'
                obj.type = val;
                obj = computeCache(obj);
                
        end
    end
end

