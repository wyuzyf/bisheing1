function  folds = Kfold(N, K)
% Kfold Compute indices for K-fold cross validaiton
% folds = Kfold(N,K)
% N = num data
% K = num folds
% folds{i} = indices of i'th fold
%
% Example:
% folds = Kfold(100, 3)
% folds{1} = 1:33, folds{2} = 34:66, folds{3} = 67:100
% (last fold gets all the left over so has different length)
%
% See also partitionData

ndx = 1;
for i=1:K
  low(i) = ndx;
  Nbin(i) = fix(N/K);
  if i==K
    high(i) = N;
  else
    high(i) = low(i)+Nbin(i)-1;
  end
  folds{i} = low(i):high(i);
  ndx = ndx+Nbin(i);
end

