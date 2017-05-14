%运动目标检测
clc
clear
FrameNum=100;
StartFrame=1;
BackGround=rgb2gray(frame2im(aviread('traffic.avi',1)));

for k=StartFrame+1:1:FrameNum
    CurrentFrame=rgb2gray(frame2im(aviread('traffic.avi',k)));
    FormerFrame=rgb2gray(frame2im(aviread('traffic.avi',k-1)));
    FD=abs(CurrentFrame-FormerFrame);                                       %帧间差分
    
    %T=threshold(FD);
    T=0.17;
    bw=im2bw(FD,T);
    a=0.1;                                                                  %背景更新因子，控制更新速度
    CurrentBack=double(BackGround).*bw+(a.*double(CurrentFrame)+(1-a).*double(BackGround)).*(1-bw);  %背景更新
    BackGround=CurrentBack;
    FD1=abs(CurrentFrame-uint8(BackGround));
    %T2=threshold(FD1);
    bw2=im2bw(FD1,T);
    SE=[1,1,1;1,1,1;1,1,1];
    bwrode=imerode(bw2,SE);
    bwdi=imdilate(bw2,SE);
    bwdi2=bwmorph(bwrode,'dilate',3);
    bwdi3=imopen(bw2,SE);

    %figure
    %subplot(1,4,1), imshow(bwdi);
    %subplot(1,4,2), imshow(bwdi2);
    %subplot(1,4,3), imshow(bwdi3);
    %subplot(1,4,4), imshow(aviread('traffic.avi',k));
    imshow(frame2im(aviread('traffic.avi',k)));
    %bwdi=imfill(bwdi,'holes');
    %imshow(bwdi);
    hold on;
    [L,num]=bwlabel(bwdi,8);
    Stats=regionprops(L,'Centroid');
    for i=1:num
        [H,V]=find(L==i);
        right=max(V);
        left=min(V);
        top=min(H);
        button=max(H);
        width=right-left+1;
        high=button-top+1;
        plot(Stats(i).Centroid(1),Stats(i).Centroid(2),'R+');
        rectangle('position',[left,top,width,high],'EdgeColor','r');
        pause(0.01);
    end
 end
