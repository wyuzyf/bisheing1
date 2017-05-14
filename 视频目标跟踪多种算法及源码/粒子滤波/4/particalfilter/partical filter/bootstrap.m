function [samples,q] = bootstrap(actualbar_pos,y,numSamples,initVar,deta,Q);
% PURPOSE : This m file performs the bootstrap algorithm (a.k.a. SIR,
%           particle filter, etc.) for the model specified in the
%           file sirdemo1.m. 
% INPUTS  : - actualx = The true hidden state. 
%           - y = The observation.
%           - R = The measurement noise variance parameter.
%           - Q = The process noise variance parameter.
%           - initVar = The initial variance of the state estimate.
%           - numSamples = The number of samples.
% OUTPUTS : - x = The estimated state samples.
%           - q = The normalised importance ratios.

% AUTHOR  : Nando de Freitas - Thanks for the acknowledgement :-)
% DATE    : 08-09-98

if nargin <3, error('Not enough input arguments.'); end

[rows,cols] = size(y);      % rows = Max number of time steps.
S = numSamples;             % Number of samples;
Nstd = 2;                   % No of standard deviations for error bars;
bar_pos=zeros(S,rows);
bar_posu=zeros(S,rows);
bar_vel=zeros(S,rows);
q=zeros(S,rows);

% SAMPLE FROM THE PRIOR:
% =====================
bar_vel(:,1) = sqrt(initVar)*randn(S,1);  %为什么是正态分布？ 计算均值和方差的作用是什么？ 
bar_pos(:,1)=mod(deta.*bar_vel(:,1),1);
mean(bar_pos(:,1));
cov(bar_pos(:,1));

figure(1)
clf;
subplot(221)
plot(actualbar_pos)
ylabel('State bar_pos','fontsize',15);
xlabel('Time','fontsize',15);

% UPDATE AND PREDICTION STAGES:
% ============================
for t=1:rows-1,
   
  figure(1)  
  subplot(222)
  plot(y)
  ylabel('Output y','fontsize',15);
  xlabel('Time','fontsize',15);
%   hold on
%   plot(t*ones(1,99),[-39:1:69],'r');
%   hold off
  subplot(223)
  hold on
  plot(t,mean(bar_pos(1:S,t)),'ro',t,actualbar_pos(t,1),'go'); %bar_pos(1:S,t,1),什么意思？
  hold on
  errorbar(t,mean(bar_pos(1:S,t)),Nstd*std(bar_pos(1:S,t)),Nstd*std(bar_pos(1:S,t)),'k')
  legend('Posterior mean estimate','True value');
  ylabel('Sequential state estimate','fontsize',15)
  xlabel('Time','fontsize',15)

  bar_posu(:,t) = predictstates(bar_vel(:,t),bar_pos(:,t),deta);
  
  q(:,t+1) = importanceweights(bar_posu(:,t),y(t+1,1),deta,Q);
  bar_pos(:,t+1) = updatestates(bar_posu(:,t),q(:,t+1));
end;
% figure(3);
% plot(1:600,bar_posu(:,1),'b*',2,actualbar_pos(2),'r+');
% figure(4);
% plot(1:600,bar_pos(:,2),'b*');









