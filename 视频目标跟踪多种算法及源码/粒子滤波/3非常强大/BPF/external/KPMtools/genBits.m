function bits = genBits(n, k)
% function bits = genbits(n, k)
% Generate bit vectors of length n with at most k bits turned on
%
% Example
%>> genBits(3,1)
%ans =
%             0             0             0
%             0             0          1.00
%             0          1.00             0
%          1.00             0             0
	  
if nargin < 2, k = n; end

bits = decodeBits(0:(2^n)-1);
bad = find(sum(bits,2) > k);
bits(bad,:) = [];
