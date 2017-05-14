function unfreezeColors(h)
% unfreezeColors  Restore colors of an image to original indexed color
%
%   Useful if you want to apply a new colormap to an image whose colors
%       were previously frozen with freezeColors
%
%   Usage:
%       unfreezeColors works on gca, unfreezeColors(axh) works on axis axh.
%
%       Only does something on axes on which freezeColors has already been
%       called.
%
%   see also freezeColors
%
%   JRI (iversen@nsi.edu) 3/23/05

if nargin < 1,
    h = gca;
end

imh = findobj(h, 'type','image');
cmap = colormap;
nColors = size(cmap,1);

for hh = imh',
    try
        ud = getappdata(hh,'JRI_freezeColorsData');
        haveAppdata = 1;
    catch
        ud = get(hh,'userdata');
        haveAppdata = 0;
    end
    cdata = get(hh,'cdata');
    if ~isempty(ud),
        indexed = ud{1};
        scalemode = ud{2};
        if all(size(indexed) == size(cdata(:,:,1))),
            set(hh,'cdata',indexed);
            set(hh,'cdatamapping',scalemode);
            if haveAppdata,
                rmappdata(hh,'JRI_freezeColorsData')
            else
                set(hh,'userdata',[]);
            end
        else
            warning('Could not restore indexed data: it is the wrong size.')
        end
    else
        disp('unfreezeColors: nothing to restore.')
    end
end %loop on images

