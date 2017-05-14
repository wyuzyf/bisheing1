%mex ndsumC.c addflops.c
x = rand([2 3 4]);
ndsum(x,[1 3])
ndsumC(x,[1 3])

if 1
  % special cases
  ndsum(x,1:3)
  ndsumC(x,1:3)
  ndsum(x,[])
  ndsumC(x,[])
end

tim = [];
for k = 1:3
  sz = repmat(10*k,1,5);
  x = rand(sz);
  niter = floor(10/k);
  
  %tic; for i = 1:niter sum(x,4); end; tim(k,3) = toc;
  tic; for i = 1:niter ndsum(x,[2 4]); end; tim(k,1) = toc;
  tic; for i = 1:niter ndsumC(x,[2 4]); end; tim(k,2) = toc;
end
figure(1)
clf
plot(tim)
xlabel('trial')
ylabel('time (sec)')
%legend('ndsum', 'ndsumC', 'sum', 2)
legend('ndsum', 'ndsumC', 2)
axis_pct
