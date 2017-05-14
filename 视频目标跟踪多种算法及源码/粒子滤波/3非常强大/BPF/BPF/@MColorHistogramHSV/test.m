function test(obj)
% TEST          Test script
%
%   Example:
%       test(MColorHistogramHSV);
%
%   See also: MCOLORHISTOGRAMHSV

% Copyright 2006 Wei-Lwun Lu
% test.m version 1.0

obj = MColorHistogramHSV('peppers.png');

test_set_get(obj);
test_constructor(obj);
test_demo(obj);
test_visualize(obj);


function test_constructor(obj)

errormsg = 'constructor fails';
fprintf('Testing constructor');

obj = MColorHistogramHSV;
assert(1, errormsg);
obj = MColorHistogramHSV('peppers.png');
assert(1, errormsg);
obj = MColorHistogramHSV('peppers.png', 15, 15, 15);
assert(get(obj, 'hbins') == 15, errormsg);
assert(get(obj, 'sbins') == 15, errormsg);
assert(get(obj, 'vbins') == 15, errormsg);
assert(get(obj, 'hsbins') == 15 * 15, errormsg);
obj = MColorHistogramHSV('peppers.png', 10, 10, 10, 'builtin');
assert(strcmp(get(obj, 'type'), 'builtin'), errormsg);
obj = MColorHistogramHSV('peppers.png', 10, 10, 10, 'local');
assert(1, errormsg);
obj = MColorHistogramHSV('peppers.png', 10, 10, 10, 'global');
assert(~isempty(obj.hsIntegralHistogram), errormsg);
assert(~isempty(obj.vIntegralHistogram), errormsg);

fprintf('Pass\n');


function test_set_get(obj)

errormsg = 'get() fails';
fprintf('Testing set() and get()');

obj = set(obj, 'hbins', 15);
assert(get(obj, 'hbins') == 15, errormsg);

obj = set(obj, 'sbins', 15);
assert(get(obj, 'sbins') == 15, errormsg);

obj = set(obj, 'vbins', 15);
assert(get(obj, 'vbins') == 15, errormsg);
assert(get(obj, 'hsbins') == 15 * 15, errormsg);

width = get(obj, 'width');
height = get(obj, 'height');
img = get(get(obj, 'image'), 'image');
assert(size(img, 1) == height, errormsg);
assert(size(img, 2) == width, errormsg);

obj = set(obj, 'type', 'builtin');
assert(strcmp(get(obj, 'type'), 'builtin'), errormsg);

img = imread('trees.tif');
obj = set(obj, 'image', MImage('trees.tif'));
assert(get(obj, 'width') == size(img, 2), errormsg);
assert(get(obj, 'height') == size(img, 1), errormsg);

fprintf('Pass\n');


function test_demo(obj)

fprintf('Testing demo()...........\n');
demo(MColorHistogramHSV, 'nWindows', 300);
demo(MColorHistogramHSV, 'nWindows', 3000);
demo(MColorHistogramHSV, 'nWindows', 10000);



function test_visualize(obj)

fprintf('Testing visualize()..........\n');
obj = MColorHistogramHSV('peppers.png');
visualize(obj);


function assert(exp, errormsg)

if ~exp
    error(errormsg)
else
    fprintf('.');
end

