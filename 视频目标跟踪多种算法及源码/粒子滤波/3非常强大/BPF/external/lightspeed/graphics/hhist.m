function h = hhist(y, r)
% HHIST       Locally-adaptive unbiased density estimate.
%
% HHIST(y,r) returns an estimate of p(y) at each of the locations given in r,
% i.e. p(y=r).
% Both y and r are vectors.
% HHIST(y) evaluates the density at 100 points along the range of y.
%
% The algorithm is linear interpolation of the empirical cumulative 
% distribution.  Reference:
% Bruce M. Hill, "Posterior Distribution of Percentiles: Bayes'
% Theorem for Sampling from a Population", Journal of the American
% Statistical Association, Vol. 63, No. 322. (Jun., 1968),
% pp. 677-691.
% http://links.jstor.org/sici?sici=0162-1459%28196806%2963%3A322%3C677%3APDOPBT%3E2.0.CO%3B2-O
%
% See test_hhist.m for a demonstration.

% Written by Tom Minka

y = y(:);
% jitter duplicated y's, because interp1 will fail if there are duplicates.
% want to keep the last duplicate.
while 1
  dup = duplicated(y);
  if ~any(dup)
    break
  end
  y = y + 1e-14*dup.*(2*rand(size(y))-1);
end

y = sort(y);

if nargin < 2
  r = linspace(min(y),max(y),100);
  q = (1:length(y))/length(y);
else
  % add extreme observations
  y = [-1e10; y; 1e10];
  q = (0:(length(y)-1))/(length(y)-1);
end

% interpolate the cumulative so we can differentiate it
r = r(:);
h = interp1(y,q',r);
h = gradient(h)./gradient(r);

if nargout == 0
  plot(r, h)
  clear h
end
