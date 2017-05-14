%========PF(just as a reference)================================
%====track the motive goal with constant speed=======
%== if initial estimated vulue is far from real initial vaule,
%==then it works badly!!
%===increase the amount of particles to update the accuracy==
clc;clear;close all;
t=50;N = 100;
x=zeros(2,t);
pred_xx=zeros(2,t);
PF_x = zeros(2,N);
pre_x=PF_x;
x(:,1)=[200;13];%initial [pos,velocity]
pred_xx(:,1)=[190;10];
P=5;R=.05;Q=.05;T=1;
F=[1 T;0 1];
for i=1:N
    PF_x(:,i)=pred_xx(:,1)+sqrt(P)*randn(2,1);
end
for k=2:t
    x(:,k) = F*x(:,k-1)+sqrt(Q)*randn(2,1);%state worthy
    y = x(1,k)+sqrt(R)*randn;
    % filter process
    for i=1:N
       pre_x(:,i)=F*PF_x(:,i)+sqrt(Q)*randn(2,1);
       Pred_y = pre_x(1,i)+sqrt(R)*randn;
       error = y-Pred_y;
       q(i)= 1/(sqrt(2*pi*R))*exp(-error^2/(2*R));%update weight
    end
    % normalization
    pf_sum=sum(q);q=q./pf_sum;
%% =========resample=============
    for i=1:N   
        p_u=rand;
        p_temp=0;
        for m=1:N
            p_temp=p_temp+q(m);
            if p_temp>=p_u
                PF_x(:,i)=pre_x(:,m);
                break;
            end
        end  
    end
   pred_xx(:,k)=mean(PF_x,2);%the mean value of each row,because all particles have the same weight.
end
Err = pred_xx-x;
figure('color','white');
subplot(211);
plot(x(1,:),'r+');
hold on;
plot(pred_xx(1,:),'y*');
legend('真实位置','粒子滤波位置估计值','location','best');
xlabel('观测次数');
ylabel('观测值');grid on;
subplot(212);
plot(x(2,:),'r+');
hold on;
plot(pred_xx(2,:),'y*');
legend('真实速度','粒子滤波速度估计值','location','best');
xlabel('观测次数');
ylabel('观测值');grid on;
figure('color','white');
plot(1:t,Err(1,:),'b-',1:t,Err(2,:),'k','linewidth',2);
legend('位置误差','速度误差');title('估计误差');grid on;