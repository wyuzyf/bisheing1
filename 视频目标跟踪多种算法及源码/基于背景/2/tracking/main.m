clear data
disp('input video');

avi = VideoReader('ÈË´©Ê÷.avi');

vidFrames = read(avi);
numFrames = avi.numberOfFrames; 
mov(1:numFrames) = struct('cdata',zeros(avi.Height,avi.Width,3,'uint8'),...
    'colormap',[]);


for k = 1 : numFrames - 1
    mov(k).cdata = vidFrames(:,:,:,k);
    mov(k).colormap = [];
    k
end

video = {mov.cdata};
for a = 1:length(video)
    imagesc(video{a});
    axis image off
    drawnow;
end;
disp('output video');
tracking(video,avi.Height,avi.Width);
