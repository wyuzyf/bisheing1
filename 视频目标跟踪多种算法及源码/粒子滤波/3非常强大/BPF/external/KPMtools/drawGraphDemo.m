N=6; Adj = rand(N) > .5;
for i=1:N, Adj(i,i)=0; end
%drawGraph(Adj)
drawGraph(Adj, {'a','b','cc','d','e','f'}) 

% Test ability to draw isolated nodes
N = 10;
Adj = zeros(N);
for i=1:N-1
  Adj(i,i+1) = 1;
end
Adj(2,3)=0; Adj(3,4)=0;
Adj(5,4) = 1; % reverse link
drawGraph(Adj)

