N = 10000;
a = 2.7;
d = 3;
m = zeros(d,d);
s = 0;
for i = 1:N
  L = randwishart(a,d);
  X = L'*L;
  m = m + X;
  s = s + logdet(X);
end
i = 0:(d-1);
sTrue = sum(digamma(a - i*0.5));
m = m/N;
s = s/N;
fprintf('Wishart(%g) mean: (should be %g*I)\n', a, a);
disp(m)
fprintf('  E[logdet]: %g (should be %g)\n', s, sTrue);
