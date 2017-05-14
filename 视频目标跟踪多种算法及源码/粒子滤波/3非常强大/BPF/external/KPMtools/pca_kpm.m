function [pc_vec]=pca_kpm(features,N, method);
% PCA_KPM Compute top N principal components using eigs or svd.
% [pc_vec]=pca_kpm(features,N) 
%
% features(:,i) is the i'th example - each COLUMN is an observation
% pc_vec(:,j) is the j'th basis function onto which you should project the data
% using pc_vec' * features

[d ncases] = size(features);
fm=features-repmat(mean(features,2), 1, ncases);


if method==1 % d*d < d*ncases
  fprintf('pca_kpm eigs\n');
  options.disp = 0;
  C = cov(fm'); % d x d matrix
  [pc_vec, evals] = eigs(C, N, 'LM', options);
else 
  % [U,D,V] = SVD(fm), U(:,i)=evec of fm fm', V(:,i) = evec of fm' fm
  fprintf('pca_kpm svds\n');
  [U,D,V] = svds(fm', N);
  pc_vec = V;
end

if 0
  N = 10; p = 3;
X = randn(N, p);
X = X-repmat(mean(X),N,1);
num_retain = rank(X);

C = (X'*X)/(N-1);
C2=cov(X);
assert(approxeq(C,C2))

[Evec, D]=eig(C);
Eval = diag(D); 

% check correctness
err = mean(mean(abs(C*Evec - Evec*D)));
assert(approxeq(0, err))

% Sort so largest eval is first
[evals,index]=sort(Eval); index=flipud(index); 
Evec = Evec(:, index); 


num_retain = rank(X);
Evec = Evec(:,1:num_retain) % some may be junk

[U2,D2,V2] = svd(X); % V2 already sorted so largest singular value first
Evec2 = V2;
Evec2 = Evec2(:,1:num_retain) % some may be junk

assert(approxeq(abs(Evec), abs(Evec2)))

end
