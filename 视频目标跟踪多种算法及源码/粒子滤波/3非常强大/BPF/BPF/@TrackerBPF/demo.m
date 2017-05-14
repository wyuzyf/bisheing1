function demo(obj, varargin)
% PURPOSE : Demonstration of @TrackerBPF
% INPUT : - obj          = @TrackerBPF object
%         - varargin     = specific parameter settings for @TrackerBPF
% AUTHOR: Kenji Okuma
% DATE: January 2007
% =========================================================================

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% demo options
% change here please
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Display option: true for displaying confidence map. false otherwise.
g_options.conf_map = false;
% Detection option: true for detecting targets online. false otherwise.
g_options.detector = true;
% Save option: true for saving the resulting movie.  false otherwise.
g_options.save_movie = false;
% write out detection results
g_options.save_detection_results = false;

% color table for drawing boxes
% (up to 16 different colors and black for targets in interactions)
r = [150,   0, 255, 255,   0,   0, 255,   0,   0, 100, 100, 100,   0, 100, 50, 50,  0];
g = [150, 255,   0, 255,   0, 255,   0,   0, 100,   0, 100,   0, 100, 100, 50,  0,  0];
b = [150, 255, 255,   0, 255,   0,   0, 100,   0,   0,   0, 100, 100, 100, 50, 50,  0];
colortable = [r', g', b'];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set up parameters if necessary
obj = set(obj, 'online_detector', g_options.detector, varargin);

% get images
DATA_PATH = sprintf('data/');
imgList = dirKPM(strcat(DATA_PATH, '*.jpg'));
numImages = length(imgList);

% file extensions: only jpg for now
INPUT_FILE_FORMAT = 'jpg';

% get the first frame
filename = sprintf('%s%s', DATA_PATH, imgList{1});
img = imread(filename, INPUT_FILE_FORMAT);
[imgHeight imgWidth colorDepth] = size(img);

% initialize detector parameters
detector_options = init_options(obj);

% image must be 320 x 240!
if imgHeight == 240 && imgWidth == 320
    cur_img = img;
else
    cur_img = imresize(img, [240 320]);
end

time = 1;
if g_options.detector 
    % use detector (This is the default)
    % NOTE: detector must be performed on 320 x 240 images
    grayImg = rgb2grayKPM(img);
    % clusters: clusters of final detections
    % detections: a set of all detections
    % confMap: confidence map of adaboost (activated by options)
    % iImage: integral image
    % iImage: square integral image
    [clusters detections confMap iImage iSquareImage]  = hockey_detector(grayImg, detector_options);
    boost_history{time} = clusters';
else
    % read in detection history 
    boost_history = read_detections(obj, 'detections.txt');
	grayImg = rgb2grayKPM(img);
    iImage = [];
    iSquareImage = [];
end

% move to the very first frame that has at least one detection
while isempty(boost_history{time})
    time = time + 1;
    if g_options.detector
        filename = sprintf('%s%s', DATA_PATH, imgList{time});
        img = imread(filename, INPUT_FILE_FORMAT);
        grayImg = rgb2grayKPM(img);
        cur_img = img;
        % clusters: clusters of final detections
        % detections: a set of all detections
        % confMap: confidence map of adaboost (activated by options)
        % iImage: integral image
        % iImage: square integral image
        [clusters detections confMap iImage iSquareImage] = hockey_detector(grayImg, detector_options);
        boost_history{time} = clusters';
    end
end

% initialize targets
obj = initialize(obj, boost_history{time}, filename);

% display the initial result with detections

figure('Name', 'Tracking Preview Window [BPF]');
scrsz = get(0,'ScreenSize');
set(gcf,'DoubleBuffer','on');
centerFigure(gcf, scrsz(3)*.9, scrsz(4)*.5);
uicontrol('String', 'Pause', 'Callback', 'set(gcf, ''UserData'', 1)', 'Position', [70 10 50 20]);
uicontrol('String', 'Close', 'Callback', 'set(gcf, ''UserData'', 999)', 'Position', [20 10 50 20]);
set(gcf, 'UserData', 0);

if g_options.conf_map
    % just draw tracking window    
    tmpColorTable = colortable;
    colortable = zeros(size(colortable));
    subplot(1,2,1); subimage(cur_img); drawTargets(obj, colortable, 'k', 2);
    title(['\bf\fontsize{14} Tracking Result']);

    [hsv_img] = drawConfMap(obj, filename);
    subplot(1,2,2); imagesc(hsv_img); axis image;
    title(['\bf\fontsize{14} HSV conf']);
    colortable = tmpColorTable;
else
    % just draw hog window only regardless of g_cue_option
    imagesc(cur_img); axis image; drawTargets(obj, colortable, 'k', 3);
    title(['\bf\fontsize{14} Tracking Result']);
end
mainTitle = sprintf('[ %d/%d ] \n', time, numImages);
suptitle(mainTitle);

if g_options.save_movie
    % create necessary directory
    dirName = sprintf('%s/out', pwd);
    mkdirKPM(dirName);
    fileName = sprintf('%s/%s.avi', dirName, obj.name);
    mov = avifile(fileName, 'fps', 10, 'quality', 100);
    F = getframe(gca);
    mov = addframe(mov,F);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start tracking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for current = time+1:numImages

    time = time+1;

    % read file
    filename = sprintf('%s%s', DATA_PATH, imgList{current});
    if obj.img_width == imgWidth && obj.img_height == imgHeight
        cur_img = imread(filename, INPUT_FILE_FORMAT);
    else
        img = imread(filename, INPUT_FILE_FORMAT);
        cur_img = imresize(img, obj.img_height/size(img,1));
    end

    % detect targets
    if g_options.detector
        grayImg = rgb2grayKPM(cur_img);
        % clusters: clusters of final detections
        % detections: a set of all detections
        % confMap: confidence map of adaboost (activated by options)
        % iImage: integral image
        % iImage: square integral image
        [clusters detections confMap iImage iSquareImage] = hockey_detector(grayImg, detector_options);
        boost_history{time} = clusters';
    end

    obj = update(obj, filename, grayImg, boost_history{time}, iImage, iSquareImage);
	
    mainTitle = sprintf('[ %d/%d ] \n', time, numImages);
    suptitle(mainTitle);
    if g_options.conf_map
        % just draw hog window only regardless of g_cue_option

        tmpColorTable = colortable;
        colortable = zeros(size(colortable));
        subplot(1,2,1); subimage(cur_img); drawTargets(obj, colortable, 'k', 2);
        title(['\bf\fontsize{14} Tracking Result']);        
        colortable = tmpColorTable;
        
        [hsv_img] = drawConfMap(obj, filename);
        subplot(1,2,2); imagesc(hsv_img); axis image;
        title(['\bf\fontsize{14} HSV conf']);
        
    else
        % just draw hog window only regardless of g_cue_option
        imagesc(cur_img); axis image; 
        drawTargets(obj, colortable, 'k', 3);
        title(['\bf\fontsize{14} Tracking Result']);
    end
    
    if g_options.save_movie
        F = getframe(gca);
        mov = addframe(mov,F);
    end

    usr_cmd = get(gcf, 'UserData');
    switch usr_cmd
        case 999
            close(gcf);
            if g_options.save_movie
                mov = close(mov);
            end
            clear;
            return;
        case 1
            waitforbuttonpress;
            set(gcf, 'UserData', 0);
    end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% finish tracking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if g_options.save_movie
    mov = close(mov);
end

if g_options.save_detection_results
   write_detections(obj, boost_history); 
end

close(gcf);
clear;


