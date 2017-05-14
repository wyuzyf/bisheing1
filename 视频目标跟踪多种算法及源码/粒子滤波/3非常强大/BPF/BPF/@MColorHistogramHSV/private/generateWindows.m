function rect = generateWindows(type, width, height, varargin)
% GENERATEWINDOWS   Generate a set of windows for testing
%   rect = generateWindows(type, width, height) generates a set of windows
%   for a width x height image. type = {'random', 'local', 'sliding
%   windows'}.
%
%   rect = generateWindows(..., 'propName', value) generates a set of
%   windows with additinal parameters with name 'propName' and <value>.
%   Possible propName includes:
%       'nWindows'      number of windows
%       'windowWidth'   width of windows
%       'windowHeight'  height of windows
%
%   Example:
%       rect = generateWindows('local', 100, 100, 'nWindows', 300);
%

% Copyright 2006 Wei-Lwun Lu
% generateWindows.m version 1.0


[nWindows, windowWidth, windowHeight] = ...
    process_options(varargin, 'nWindows', 1000, 'windowWidth', 30, ...
                    'windowHeight', 30);
                
switch type
    case 'random'
        rect = generateRandomWindows(width, height, nWindows, ...
                                     windowWidth, windowHeight);
    case 'local'
        rect = generateLocalWindows(width, height, nWindows, ...
                                    windowWidth, windowHeight);
    case 'sliding windows'
        rect = generateSlidingWindows(width, height, ...
                                      windowWidth, windowHeight);
    otherwise
        error('Unknown type.');
end

function rect = generateRandomWindows(width, height, nWindows, ...
                                      windowWidth, windowHeight)
% generate randomly distributed windows
%
                                  
x = ceil(rand([nWindows, 1]) * (width - windowWidth));
y = ceil(rand([nWindows, 1]) * (height - windowHeight));
rect = [x, y, x+windowWidth, y+windowHeight];
                                      
                                      

function rect = generateLocalWindows(width, height, nWindows, ...
                                     windowWidth, windowHeight)
% generate local clustered windows
%
                                 
x = ceil(10 * randn([nWindows, 1]) + (width - windowWidth) / 2);
y = ceil(10 * randn([nWindows, 1]) + (height - windowHeight) / 2);
rect = [x, y, x+windowWidth, y+windowHeight];


                                      
function rect = generateSlidingWindows(width, height, ...
                                       windowWidth, windowHeight)
% generate sliding windows
%
                                      
iRect = 1;
for y = 1 : 5 : height-windowHeight
    for x = 1 : 5 : width-windowWidth
        rect(iRect, :) = [x, y, x+windowWidth, y+windowHeight];
        iRect = iRect + 1;
    end
end

