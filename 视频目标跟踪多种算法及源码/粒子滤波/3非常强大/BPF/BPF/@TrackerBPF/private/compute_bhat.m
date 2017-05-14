function dists = compute_bhat(hist1, hist2)
% PURPOSE : compute bhattacharya distance between target and candidate
% INPUT :  - hist1        = histogram dim x p where p is the number of
%                                                   histograms
%          - hist2        = histogram dim x p
% OUTPUT : - dists        = matching distance(s)
% AUTHOR : Kenji Okuma
% DATE : January 2007
% =========================================================================

numDim = length(size(hist1));
numHists = size(hist1, numDim);

dists = zeros(1,numHists);
for count = 1:numHists
    dist = 1 - sqrt(hist1(:,count)')*sqrt(hist2(:,count));
    dists(count) = dist;
end


