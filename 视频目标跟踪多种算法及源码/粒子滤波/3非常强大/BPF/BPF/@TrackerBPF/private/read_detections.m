function boost_history = read_boost(obj, filename, maxlinelen)
% PURPOSE : Parse a text file that contains a history of detection results
% INPUT : - obj        = @TrackerBPF object
%         - filename   = name of the file
%         - maxlinelen = maximum line length (default: maxlinelen = 10000)
% OUTPUT : - boost_history = detection history 
% NOTE: file has the following format
% e.g., boost_example.txt contains:
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
% DATE: January 2007
% =========================================================================

% Returns a cell array with the lines of the file
if nargin < 3,
  maxlinelen = 10000;
end
lines = textread(filename, '%s', 'delimiter', '\n', 'bufsize', maxlinelen);

times = length(lines);
for i = 1:times,
  tmp_line = lines{i};
  numboost = 0;
  numfigure = 0;
  last = 1;
  current = [];
  for j = 1:length(tmp_line),
    if tmp_line(j) == ' ',
      numfigure = numfigure + 1;
      numboost = ceil(numfigure/3);
      entry = mod(numfigure, 3);
      if entry == 0,
        entry = 3;
      end
      current(entry, numboost) = str2num(tmp_line(last:j-1));
      last = j + 1;
    end
  end
  if size(current, 1) > 0
      current = current*obj.img_width/320;
  end
  boost_history{i} = current;
end