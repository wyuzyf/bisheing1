% PURPOSE : To estimate the states of the following nonlinear, 
%           nonstationary state space model:
%           x(t+1) = 0.5x(t) + [25x(t)]/[(1+x(t))^(2)] 
%                    + 8cos(1.2t) + process noise
%           y(t) =  x(t)^(2) / 20  + measurement noise
%           using the sequential importance-samping resampling
%           algorithm.                           
             
% AUTHOR  : Nando de Freitas - Thanks for the acknowledgement :-)
% DATE    : 08-09-98


clear;
echo off;

% INITIALISATION AND PARAMETERS:
% =============================

N = 50;                % Number of time steps.
t = 1:1:N;             % Time.  
x0 = 0;              % Initial state.
x = zeros(N,1);        % Hidden states. 
y = zeros(N,1);        % Observations.
x(1,1) = x0;           % Initial state. 
R = 1;                 % Measurement noise variance.
Q = 10;                % Process noise variance.
actv = 0.1;
initv = 0.1;
initVar = 5;           % Initial variance of the states.  
numSamples=500;        % Number of Monte Carlo samples per time step. 


% GENERATE PROCESS AND MEASUREMENT NOISE:
% ======================================

v = randn(N,1);
w = sqrt(10)*randn(N,1);
figure(1)
clf;
subplot(221)
plot(v);
ylabel('Measurement Noise','fontsize',15);
xlabel('Time');
subplot(222)
plot(w);
ylabel('Process Noise','fontsize',15);
xlabel('Time');


% GENERATE STATE AND MEASUREMENTS:
% ===============================

y(1,1) = (x(1,1)^(2))./20 + v(1,1); 
for t=2:N,
  x(t,1) = x(t-1,1)+actv+w(t,1);
  y(t,1) = (x(t,1).^(2))./20 + v(t,1); 
end;

figure(1)
subplot(223)
plot(x)
ylabel('State x','fontsize',15);
xlabel('Time','fontsize',15);
subplot(224)
plot(y)
ylabel('Observation y','fontsize',15);
xlabel('Time','fontsize',15);
fprintf('Press a key to continue')  
pause;
fprintf('\n')
fprintf('Simulation in progress')
fprintf('\n')

% PERFORM SEQUENTIAL MONTE CARLO FILTERING:
% ========================================

[samples,q] = bootstrap(x,y,R,Q,initVar,numSamples);

% COMPUTE CENTROID, MAP AND VARIANCE ESTIMATES:
% ============================================

% Posterior mean estimate
prediction = mean(samples);

% Posterior peak estimate
bins = 20;
xmap=zeros(N,1);
for t=1:N
  [p,pos]=hist(samples(:,t,1),bins);
  map=find(p==max(p));
  xmap(t,1)=pos(map(1));
end;

% Posterior standard deviation estimate
xstd=std(samples);

% PLOT RESULTS:
% ============

figure(2)

subplot(221)
plot(1:length(x),x,'g-*',1:length(x),prediction,'r-+',1:length(x),xmap,'m-*')
legend('True value','Posterior mean estimate','MAP estimate');
ylabel('State estimate','fontsize',15)
xlabel('Time','fontsize',15)
subplot(222)
plot(prediction,x,'+')
ylabel('True state','fontsize',15)
xlabel('Posterior mean estimate','fontsize',15)
hold on
c=-25:1:25; 
plot(c,c,'r');
axis([-25 25 -25 25]);
hold off
subplot(223)
plot(xmap,x,'+')
ylabel('True state','fontsize',15)
xlabel('MAP estimate','fontsize',15)
hold on
c=-25:1:25;
plot(c,c,'r');
axis([-25 25 -25 25]);
hold off
% Plot histogram:
[rows,c]=size(q);
domain = zeros(rows,1);
range = zeros(rows,1);
parameter = 1;
bins = 10;
support=[-20:1:20];

subplot(224)
hold on
ylabel('Time','fontsize',15)
xlabel('Sample space','fontsize',15)
zlabel('Posterior density','fontsize',15)
v=[0 1];
caxis(v);
for t=1:2:N,
  [range,domain]=hist(samples(:,t,parameter),support);
  waterfall((domain),t,range)
end;
















