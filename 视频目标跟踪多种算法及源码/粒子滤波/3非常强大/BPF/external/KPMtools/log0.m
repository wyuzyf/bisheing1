function [l0] = log0(a)
a(find(a==0)) = 1;
l0 = log(a);
