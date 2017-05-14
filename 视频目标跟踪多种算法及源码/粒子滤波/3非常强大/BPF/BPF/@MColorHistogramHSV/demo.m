function demo(obj, varargin)
% DEMO          Demo the @MColorHistogramGray object
%   demo(obj) demostrates the @MColorHistogramGray object.
%
%   Example:
%     demo(MColorHistogramGray);
%
%   See also MCOLORHISTOGRAMGRAY

% Copyright 2006 Wei-Lwun Lu
% demo.m version 1.0

[nWindows, hbins, sbins, vbins, infile] = ...
    process_options(varargin, 'nWindows', 10000, 'hbins', 10, ...
                    'sbins', 10, 'vbins', 10, ...
                    'infile', 'peppers.png');

windowWidth  = 30;
windowHeight = 30;

exp1(infile, nWindows, windowWidth, windowHeight, hbins, sbins, vbins);
exp2(infile, nWindows, windowWidth, windowHeight, hbins, sbins, vbins);


function exp1(infile, nWindows, windowWidth, windowHeight, hbins, sbins, vbins)
% exp1
%   randomly distributed windows

% generate windows
img = imread(infile);
img = im2double(img);
height = size(img, 1);
width  = size(img, 2);

fprintf('=====================================\n');
fprintf('Randomly distributed windows\n');

rect = generateWindows('random', width, height, 'nWindows', nWindows, ...
                       'windowWidth', windowWidth, 'windowHeight', windowHeight);
fprintf('Test %d %dx%d windows with (H, S, V) = (%d, %d, %d) bins\n', ...
        nWindows, windowWidth, windowHeight, hbins, sbins, vbins);

% compute the color histogram using Matlab builtin
t = cputime;
obj = MColorHistogramHSV(infile, hbins, sbins, vbins, 'builtin');

[hshist1, vhist1] = gethistogram(obj, rect, 1);

fprintf('Matlab Builtin CPU Time: %f sec\n', cputime - t);

% compute the color histogram using a global integral histogram
t     = cputime;
obj   = MColorHistogramHSV(infile, hbins, sbins, vbins, 'global');

[hshist2, vhist2] = gethistogram(obj, rect, 1);

fprintf('Global method CPU Time: %f sec\n', cputime - t);

if ~approxeq(hshist1, hshist2) || ~approxeq(vhist1, vhist2)
    error('Global method produces incorrect answer');
end

% compute the color histogram using a local integral histogram
t  = cputime;
obj = MColorHistogramHSV(infile, hbins, sbins, vbins, 'local');

[hshist3, vhist3] = gethistogram(obj, rect, 1);

fprintf('Local method CPU Time: %f sec\n', cputime - t);

if ~approxeq(hshist1, hshist3) || ~approxeq(vhist1, vhist3)
    error('Local method produces incorrect answer');
end

fprintf('\n\n');



function exp2(infile, nWindows, windowWidth, windowHeight, hbins, sbins, vbins)
% exp2
%   clustered windows

% generate windows
img = imread(infile);
img = im2double(img);
height = size(img, 1);
width  = size(img, 2);

fprintf('======================================\n');
fprintf('Clustered windows\n');

rect = generateWindows('local', width, height, 'nWindows', nWindows, ...
                       'windowWidth', windowWidth, 'windowHeight', windowHeight);
fprintf('Test %d %dx%d windows with (H, S, V) = (%d, %d, %d) bins\n', ...
        nWindows, windowWidth, windowHeight, hbins, sbins, vbins);

% compute the color histogram using Matlab builtin
t = cputime;
obj = MColorHistogramHSV(infile, hbins, sbins, vbins, 'builtin');

[hshist1, vhist1] = gethistogram(obj, rect, 1);

fprintf('Matlab Builtin CPU Time: %f sec\n', cputime - t);

% compute the color histogram using a global integral histogram
t     = cputime;
obj   = MColorHistogramHSV(infile, hbins, sbins, vbins, 'global');

[hshist2, vhist2] = gethistogram(obj, rect, 1);

fprintf('Global method CPU Time: %f sec\n', cputime - t);

if ~approxeq(hshist1, hshist2) || ~approxeq(vhist1, vhist2)
    error('Global method produces incorrect answer');
end

% compute the color histogram using a local integral histogram
t  = cputime;
obj = MColorHistogramHSV(infile, hbins, sbins, vbins, 'local');

[hshist3, vhist3] = gethistogram(obj, rect, 1);

fprintf('Local method CPU Time: %f sec\n', cputime - t);

if ~approxeq(hshist1, hshist3) || ~approxeq(vhist1, vhist3)
    error('Local method produces incorrect answer');
end

fprintf('\n\n');

