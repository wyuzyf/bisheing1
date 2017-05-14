x = [-Inf; -Inf];
logsumexp(x)
%logsumexpv(x)

x = rand(1000,1);
tic; for iter = 1:1000 logsumexp(x); end; toc
%tic; for iter = 1:1000 logsumexpv(x); end; toc
