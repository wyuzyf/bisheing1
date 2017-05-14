function test_sameobject
% Result should be 1 in both cases below.

a = rand(4);
b = a;
sameobject(a,b)
helper(a,a)

function x = helper(a,b)

x = sameobject(a,b);
