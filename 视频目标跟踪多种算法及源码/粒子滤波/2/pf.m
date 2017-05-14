%========PF=======
tic;
clc;clear;close all;
t=50;%length of time
N = 50;%particle
x=zeros(1,t);
PF_x=zeros(1,N);
p_xpat=PF_x;
x(1)=.6;%初始值
pred_x=2; %prediction
pred_xx=zeros(1,t);
pred_xx(1)=pred_x;
P=1;R=.002;Q=.002;
for i=1:N
    PF_x(i)=pred_x+sqrt(P)*randn;
end
for k=2:t
    x(k) = sin(x(k-1))+sqrt(Q)*randn;%state worthy
    y = x(k)^2+x(k)+sqrt(R)*randn;%measure
    % filter process
    for i=1:N
    p_xpat(i)=sin(PF_x(i))+sqrt(Q)*randn;
    Pred_y = p_xpat(i)^2+p_xpat(i)+sqrt(R)*randn;
    err_or=y-Pred_y;
    q(i) = 1/(sqrt(R*2*pi))*exp(-err_or^2/(2*R));%update weight
    end
     pf_sum=sum(q);
    q = q./pf_sum;%归一化
%% =====resample
     for i=1:N
         PF_x(i)= p_xpat(find(rand<=cumsum(q),1));%若q=[a b c d],then cumsum(q)=[a a+b a+b+c a+b+c+d],find( ,1) 返回满足条件的第一个值的索引 
     end
%%
%因为重采样之后权重都是1/N，所以粒子的加权和就是求平均
   pred_xx(k)=mean(PF_x);
%% =================
end
Err = pred_xx-x;
figure('color','white');
subplot(211);
plot(x,'r+');
hold on;
plot(pred_xx,'y*');
legend('真实估计','粒子滤波估计值','location','north');
xlabel('观测次数');
ylabel('观测值');grid on;
subplot(212);
plot(Err,'b-','linewidth',2);
legend('误差');title('估计误差');grid on;
time= toc;