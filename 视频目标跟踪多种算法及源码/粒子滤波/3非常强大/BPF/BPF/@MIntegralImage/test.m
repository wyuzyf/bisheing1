function obj = test(obj)
% TEST          Test script for @MIntegralImage
%   obj = test(obj);
%
%   Example:
%     test(MIntegralImage);
%
%   See also: MINTEGRALIMAGE

% Copyright 2006 Wei-Lwun Lu
% test.m version 1.0

obj = MIntegralImage;

test_get_set(obj);
test_constructor(obj);
test_demo(obj);


function test_get_set(obj)

errormsg = 'get() or set() tests fail!';

fprintf('Testing get() and set()');
obj = MIntegralImage(magic(10));
assert(get(obj, 'width') == 10, errormsg);
assert(get(obj, 'height') == 10, errormsg);
img = get(obj, 'integralImage');
assert(size(img, 1) == 10, errormsg);
assert(size(img, 2) == 10, errormsg);

obj = set(obj, 'data', magic(15));
assert(get(obj, 'width') == 15, errormsg);
assert(get(obj, 'height') == 15, errormsg);
img = get(obj, 'integralImage');
assert(size(img, 1) == 15, errormsg);
assert(size(img, 2) == 15, errormsg);

fprintf('Pass\n');



function test_constructor(obj)

errormsg = 'Constructor tests fail!';

fprintf('Testing constructor');
obj = MIntegralImage;
assert(1, errormsg);
obj = MIntegralImage(magic(10));
assert(get(obj, 'width') == 10, errormsg);
assert(get(obj, 'height') == 10, errormsg);
fprintf('Pass\n');


function test_demo(obj)

fprintf('Testing demo function\n');
demo(MIntegralImage);



function assert(exp, errormsg)

if exp
    fprintf('.');
else
    error(errormsg);
end
    