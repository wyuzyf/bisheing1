function val = get(obj, propName)
% GET           Get the properties of the @MColorHistogramHSV object
%   get(obj) shows the properties that are allowed to get.
%
%   val = get(obj, 'propName') get the properties of the 
%   @MColorHistogramHSV object. Possible 'propName' includes:
%     'hbins'   number of bins in H channel
%     'sbins'   number of bins in S channel
%     'vbins'   number of bins in V channel
%     'height'  height of the image
%     'width'   width of the image
%     'image'   get the gray image as a @MImage object
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
%     val = get(obj, 'nbins');
%
%   See also MCOLORHISTOGRAMHSV, MIMAGE, SET

%
% Copyright 2006 Wei-Lwun Lu
% get.m version 1.0

if nargin == 1
    strlist = {'height', 'width', 'image', 'hbins', 'sbins', ...
               'vbins', 'hsbins', 'type'};
    displayProperties(strlist);
else
    switch propName
        % image information
        case 'height'
            val = obj.height;
        case 'width'
            val = obj.width;
        case 'image'
            val = obj.MImage;
        case 'type'
            val = obj.type;
        case 'hbins'
            val = obj.hbins;
        case 'sbins'
            val = obj.sbins;
        case 'vbins'
            val = obj.vbins;
        case 'hsbins'
            val = obj.hsbins;

        otherwise
            error([propName,' Is not a valid @MColorHistogramHSV property'])
    end
end
