% extracts the center (cc,cr) and radius of the largest blob
function [cc,cr,radius,flag]=extractball(Imwork,Imback,index)%,fig1,fig2,fig3,fig15,index)
  
  cc = 0;
  cr = 0;
  radius = 0;
  flag = 0;
  [MR,MC,Dim] = size(Imback);%背景的尺寸

  % subtract background & select pixels with a big difference
  fore = zeros(MR,MC);          %image subtracktion
  fore = (abs(Imwork(:,:,1)-Imback(:,:,1)) > 10) ...
     | (abs(Imwork(:,:,2) - Imback(:,:,2)) > 10) ...
     | (abs(Imwork(:,:,3) - Imback(:,:,3)) > 10);  

  % Morphology Operation  erode to remove small noise
  foremm = bwmorph(fore,'erode',2); %2 time%数学形态学消除小噪声

  % select largest object
  labeled = bwlabel(foremm,4);%四邻域标号
  stats = regionprops(labeled,['basic']);%basic mohem nist
  [N,W] = size(stats);%总共有N个目标在图像中
  if N < 1%如果没有目标就返回
    return   
  end

  % do bubble sort (large to small) on regions in case there are more than 1排序
  id = zeros(N);
  for i = 1 : N
    id(i) = i;
  end
  for i = 1 : N-1
    for j = i+1 : N
      if stats(i).Area < stats(j).Area%将stasts（各目标）按照从大到小的顺序排序
        tmp = stats(i);
        stats(i) = stats(j);
        stats(j) = tmp;
        tmp = id(i);
        id(i) = id(j);
        id(j) = tmp;%同样id中的标号为面积大到小的区域的标号
      end
    end
  end

  % make sure that there is at least 1 big region
  if stats(1).Area < 100 %保证至少有一个足够大的区域
    return%如果最大的区域面积都小于100，则返回
  end
  selected = (labeled==id(1));%selected为0、1矩阵，1对应着id(1)的目标，即面积最大的目标

  % get center of mass and radius of largest
  centroid = stats(1).Centroid;%最大目标的中心
  radius = sqrt(stats(1).Area/pi);%最大目标的面积
  cc = centroid(1);%
  cr = centroid(2);
  flag = 1;
  return