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

N =100;                % Number of time steps.
% t = 1:1:N;             % Time.  
% x0 = 0.1;              % Initial state.
% x = zeros(N,1);        % Hidden states. 
% y = zeros(N,1);        % Observations.
% x(1,1) = x0;           % Initial state. 
% R = 1;                 % Measurement noise variance.
% Q = 10;                % Process noise variance.      
initVar = 5;           % Initial variance of the states.  
numSamples=600;        % Number of Monte Carlo samples per time step. 
deta=0.02;
sigama=0.01;
% pr=0.5;
Q=10;
bar_pos=zeros(N,1);
bar_vel=zeros(N,1);
% S=[0,1];
% r=zeros(N,1);
y=zeros(N,1);

%节奏模式函数
% n=0:4;
% u=(n>=0);



% GENERATE PROCESS AND MEASUREMENT NOISE:
% ======================================

% v = randn(N,1);
% w = sqrt(10)*randn(N,1);
% figure(1)
% clf;
% subplot(221)
% plot(v);
% ylabel('Measurement Noise','fontsize',15);
% xlabel('Time');
% subplot(222)
% plot(w);
% ylabel('Process Noise','fontsize',15);
% xlabel('Time');


% GENERATE STATE AND MEASUREMENTS:
% ===============================

% y(1,1) = (x(1,1)^(2))./20 + v(1,1); 
% for t=2:N,
%   x(t,1) = 0.5*x(t-1,1) + 25*x(t-1,1)/(1+x(t-1,1)^(2)) + 8*cos(1.2*(t-1)) + w(t,1);
%   y(t,1) = (x(t,1).^(2))./20 + v(t,1); 
% end;
y=[
0.25
0.5
0.75
1
1.25
1.5
1.75
2
2.125
2.25
2.375
2.5
2.75
3
3.25
3.5
3.75
4
4.125
4.25
4.375
4.5
4.75
5
5.25
5.5
5.75
6
6.25
6.5
6.75
7
7.25
7.5
7.75
8
8.25
8.25
8.5
8.75
9
9.25
9.5
9.75
10
10.125
10.25
10.375
10.5
10.75
11
11.25
11.5
11.75
12
12
12.25
12.5
12.75
13
13.25
13.5
13.75
14
14
14
14.25
14.5
14.75
15
15
15
15.25
15.5
15.5
15.75
16
16
16
16.25
16.5
16.75
17
17
17.25
17.5
17.75
18
18
18
18
18.25
18.5
18.75
19
19.25
19.5
19.75
20
20.25
];
bar_vel(1,1)=1.5;
for t=2:N
bar_vel(t,1)=bar_vel(t-1)+sigama*randn(1,1);
bar_pos(t,1)=mod((bar_pos(t-1,1)+deta*bar_vel(t-1,1)),1);
end


figure(1)

subplot(222)
plot(bar_vel)
ylabel('State bar_vel','fontsize',15);
xlabel('Time','fontsize',15);
subplot(223)
plot(bar_pos)
ylabel('State bar_pos','fontsize',15);
xlabel('Time','fontsize',15);
subplot(224)
plot(y,'o')
ylabel('Observation y','fontsize',15);
xlabel('Time','fontsize',15);
% fprintf('Press a key to continue')  
% pause;
% fprintf('\n')
% fprintf('Simulation in progress')
% fprintf('\n') 

% PERFORM SEQUENTIAL MONTE CARLO FILTERING:
% ========================================

% [samples,q] = bootstrap(x,y,R,Q,initVar,numSamples);
[samples,q] = bootstrap(bar_pos,y,numSamples,initVar,deta,Q);
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
















