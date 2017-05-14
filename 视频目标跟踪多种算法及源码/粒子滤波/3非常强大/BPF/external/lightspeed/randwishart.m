function cholX = randwishart(a,d)
%RANDWISHART    Sample from Wishart distribution.
%
% cholX = RANDWISHART(A,D) returns a DxD upper triangular matrix such that
% cholX'*cholX is a sample from a Wishart distribution with shape parameter
% A and unit scale.

% Written by Tom Minka
% (c) Microsoft Corporation. All rights reserved.

sqrth = 0.70710678118655;  % sqrt(0.5)
cholX = triu(randn(d,d))*sqrth;
i = 0:(d-1);
cholX(finddiag(cholX)) = sqrt(randgamma(a - i*0.5));
