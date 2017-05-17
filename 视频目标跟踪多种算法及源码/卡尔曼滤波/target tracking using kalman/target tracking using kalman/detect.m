%detect
clear,clc%清屏
% compute the background image
Imzero = zeros(240,320,3);%背景初始化
for i = 1:5
Im{i} = double(imread(['DATA/',int2str(i),'.jpg']));
Imzero = Im{i}+Imzero;
end
Imback = Imzero/5;%五帧平均化为背景模板，因为本文用的是背景差方法检测运动目标的
[MR,MC,Dim] = size(Imback);%三维图像的尺寸

% loop over all images
for i = 1 : 60
  % load image
  Im = (imread(['DATA/',int2str(i), '.jpg'])); 
  imshow(Im)
  Imwork = double(Im);

  %extract ball
  [cc(i),cr(i),radius,flag] = extractball(Imwork,Imback,i);%,fig1,fig2,fig3,fig15,i);%检测运动的球
  if flag==0
    continue
  end
    hold on
    for c = -0.9*radius: radius/20 : 0.9*radius
      r = sqrt(radius^2-c^2);
      plot(cc(i)+c,cr(i)+r,'g.')
      plot(cc(i)+c,cr(i)-r,'g.')
    end
 %Slow motion!
      pause(0.02)
end

figure

  plot(cr,'g*')
  hold on
  plot(cc,'r*')