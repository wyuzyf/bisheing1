function p = sigmoid(x)
% p(i) = sigma(x(i) = 1/(1+e^{-x(i)})

p = 1 ./ (1+exp(-x));
