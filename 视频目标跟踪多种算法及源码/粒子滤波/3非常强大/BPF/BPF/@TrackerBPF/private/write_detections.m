function write_detections(obj, boost_history, filename)
% PURPOSE : Save a text file that contains a history of detection results
% INPUT : - obj           = @TrackerBPF object
%         - boost_history = detection history
%         - filename      = name of the file to save the history
% OUTPUT : 
% NOTE: file has the following format
% e.g., the saved file contains:
%
%       x1_1 y1_1 s1_1 x1_2 y1_2 s1_2
%       x2_1 y2_1 s2_1 x2_2 y2_2 s2_2 x2_3 y2_3 s2_3
%       ...
%
%       the above entries mean that there are two targets in the first
%       frame and three targets in the second frame and so on. (x y)
%       indicates the upper left corner of the detection box and s
%       reprensents the scale of the box.  The origial width and height
%       of the box should be gvien with @TrackerBPF object.
% AUTHORS: Kenji Okuma
% DATE: February 2007
% =========================================================================

% Returns a cell array with the lines of the file
if nargin < 2
    filename = 'detections.txt';
end
if ~obj.online_detector
    FID = fopen(filename, 'w');
    times = length(boost_history);
    for i = 1:times,
        current_targets = boost_history{i};
        for j = 1:size(current_targets,2)
            fprintf(FID, '%d %d %f ', current_targets(1,j), current_targets(2,j), current_targets(3,j));
        end
        fprintf(FID, '\n');
    end
    fclose(FID);
end