% timing test for normpdf

niter = 1000;
d = 100;
A = rand(d);
A = A*A';
S = chol(A);
iA = inv(A);
iS = chol(iA);
x = rand(d,1);

fprintf('time for standard normal:')
tic;for i = 1:niter normpdfln(x); end;toc
fprintf('time for single mean:')
tic;for i = 1:niter normpdfln(x,x(:,1)); end;toc
fprintf('time for zero mean, Cholesky argument:')
tic;for i = 1:niter normpdfln(x,[],S); end;toc
fprintf('time for Cholesky argument:')
tic;for i = 1:niter normpdfln(x,x,S); end;toc
fprintf('time for variance argument:')
tic;for i = 1:niter normpdfln(x,x,[],A); end;toc
fprintf('time for inverse Cholesky argument:')
tic;for i = 1:niter normpdfln(x,x,iS,'inv'); end;toc
fprintf('time for inverse variance argument:')
tic;for i = 1:niter normpdfln(x,x,'inv',iA); end;toc

