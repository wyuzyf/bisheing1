function centerFigure(handles, width, height)

if isempty(gcbf)
    ScreenUnits=get(0,'Units');
    set(0,'Units','pixels');
    ScreenSize=get(0,'ScreenSize');
    set(0,'Units',ScreenUnits);

    FigPos(1)=1/2*(ScreenSize(3)-width);
    FigPos(2)=1/2*(ScreenSize(4)-height);
else
    GCBFOldUnits = get(gcbf,'Units');
    set(gcbf,'Units','pixels');
    GCBFPos = get(gcbf,'Position');
    set(gcbf,'Units',GCBFOldUnits);
    FigPos(1:2) = [(GCBFPos(1) + GCBFPos(3) / 2) - width / 2, ...
                   (GCBFPos(2) + GCBFPos(4) / 2) - height / 2];
end
FigPos(3:4)=[width height];
set(handles, 'Position', FigPos);