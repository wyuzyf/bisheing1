x = -20:2;
w = exp(-0.5*x.^2 -0.5*log(2*pi));
f = exp(-normcdfln(x));
g = 1./normcdf(x);
%g = exp((log(1+exp(0.88+x))./1.5).^2);
%g = exp(0.5*log(2*pi) +0.5*x.^2 + log(x));
plot(x, f.*w, x, g.*w)

if 0
% test approximations
a = exp(sqrt(2/pi));
b = 1/log(2/pi*a);
g = log(a-1 + exp(x.*exp(1./(x.^2 + b))));
g = log(a-1 + exp(x));
plot(x, f, x, g)
plot(x, log(exp(f)+1-a)./x)
plot(x, 1./log(log(exp(f)+1-a)./x))
end

% read `/u/tpminka/src/maple/gauss_cdf`; 
% f := 1/sqrt(2*Pi)*exp(-1/2*x^2)/gauss_cdf(-x);
% g := exp(sqrt(2/Pi))-1+exp(x);
% plot(f, x=0..50);
% h := log(exp(f*sqrt(Pi/2)-1)+1);
% h := log(exp(f - sqrt(2/Pi))+1);
% h := 1/log(log(exp(f) +1-exp(sqrt(2/Pi)))/x);
% asympt(h,x);
