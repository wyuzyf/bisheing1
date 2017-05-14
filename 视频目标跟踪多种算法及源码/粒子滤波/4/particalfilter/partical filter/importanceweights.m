function q = importanceweights(bar_posu,y,deta,Q);
% PURPOSE : Computes the normalised importance ratios for the 
%           model described in the file sirdemo1.m.
% INPUTS  : - xu = The predicted state samples.
%           - y = The output measurements.
%           - R = The measurement noise covariance.
% OUTPUTS : - q = The normalised importance ratios.

% AUTHOR  : Nando de Freitas - Thanks for the acknowledgement :-)
% DATE    : 08-09-98


if nargin < 4, error('Not enough input arguments.'); end

[rows,cols] = size(bar_posu);
q = zeros(size(bar_posu));
w = zeros(size(bar_posu));
u = zeros(size(bar_posu));
a = zeros(size(bar_posu));
b = zeros(size(bar_posu));
yy=0;
wsum=0;
% m = (barposu.^(2))./20;
%节奏模式函数
% t=0:0.01:1;
%  u = tripuls(t,0.05,0)+tripuls(t-0.25,0.05,0)+tripuls(t-0.5,0.05,0)+tripuls(t-0.75,0.05,0)+tripuls(t-1,0.05,0);
u = tripuls(bar_posu,0.05,0)+tripuls(bar_posu-0.25,0.05,0)+tripuls(bar_posu-0.5,0.05,0)+tripuls(bar_posu-0.75,0.05,0)+tripuls(bar_posu-1,0.05,0);
a=(u.^2)/Q;
b=u./Q;
yy=factorial(floor(y));
%权值的和
% % w=sum((((b.^a)).*gamma(a.+y.*ones(rows,cols)))./((yy.*ones(rows,cols)).*gamma(a).*((b.+deta.*ones(rows,cols)).^(a.+y.*ones(rows,cols)))));
% % w=((b.^a).*gamma(a.+y.*ones(rows,cols)))./((yy.*ones(rows,cols)).*gamma(a).*(b.+deta.*ones(rows,cols)).^(a.+y.*ones(rows,cols)));
% t1=(b.^a).*gamma(a.+y.*ones(size(bar_posu)));
% t2=(yy.*ones(size(bar_posu))).*gamma(a);
% t3=(b.+deta.*ones(size(bar_posu))).^(a.+y.*ones(size(bar_posu)));
% sum(t1./(t2.*t3));
% sumw=cumsum(w);
for s=1:rows
%   q(s,1) = exp(-.5*R^(-1)*(y-
%   m(s,1))^(2))./sum(exp(-.5*R^(-1)*(y.*ones(size(xu))-m).^(2))); 为什么是这样？
w(s,1)=((b(s,1)^a(s,1))*gamma(a(s,1)+y))./(yy*gamma(a(s,1))*((b(s,1)+deta)^(a(s,1)+y)));
end;
wsum=sum(w);
for s=1:rows
    q(s,1)=w(s,1)/wsum;
end

subplot(224); 
plot(bar_posu,q,'+')      
ylabel('Likelihood function','fontsize',15);
xlabel('Hidden state support','fontsize',15)
% axis([-30 30 0 0.03]);

