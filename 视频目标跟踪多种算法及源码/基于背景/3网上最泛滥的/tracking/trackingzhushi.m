                 %%%%%%%%%%   Directions:  % 表示对程序语句的注释; %%   表示对程序注释语句的注释的注释. %%%%%%%%%%
                                        

function d = tracking(video)                        % tracking函数定义.
if ischar(video)                                    % Determine whether item is character array，tf = ischar(A) returns logical 1 (true).
                                                    % if A is a character array and logical 0 (false) otherwise.
                                                    % 判断判断video是否为一个字符数组。是返回1，否则返回0.
                                                    
                                                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  读取所需视频信息  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Load the video from an avi file.
    %%从一个avi文件中读取视频文件,获取相关视频文件信息.
    
    avi = aviread(video);                           % mov = aviread(filename)  reads the AVI movie filename into the MATLAB movie structure mov。
                                                    % 该函数将avi中的视频信息读入到MATLAB电影缓存中。
    pixels = double(cat(4,avi(1:2:end).cdata))/255; % double(x) returns the double-precision value for X. If X is already a double-precision
                                                    % array, double has no effect.如果X是一个非双精度数值则该函数返回一个双精度值，否则该函数无任何作用.
                                                    % cat：Concatenate arrays along specified dimension沿具体指定维将avi视频数据分块为由4个矩阵元素组成的矩阵
                                                    
                                                    
                                                    
    clear avi                                       % clear name :removes just the M-file or MEX-file function or variable name from the workspace.
                                                    % 清除工作空间缓存。

else
    
    % Compile the pixel data into a single array
    %% 把像素数据编译成为一个单独的4维数组
    
    pixels = double(cat(4,video{1:2:end}))/255;     % 像素值pixels定义为矩阵。
    clear video                                     % clear keyword: clears the items indicated by keyword.按关键字video清除各项目
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   图像转化   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Convert to RGB to GRAY SCALE image.
% % 把RGB转换为灰度图像

nFrames = size(pixels,4);                           % m = size(X,dim) :returns the size of the dimension of X specified by scalar dim.
                                                    % 下标dim指示pixdls数组的维数值dim大小，即将dim赋值给nFrames。 
                                                    
for f = 1:nFrames                                   % 视频图像帧循环变量f

%     F = getframe(gcf);                            %%  F = getframe(gcf):gets a frame from the figure or axes identified by handle h.
                                                    %% 从由h确定的图片或者轴线获得一帧。
%     [x,map]=frame2im(F);                          %% [X,Map] = frame2im(F) :converts the single movie frame F into the indexed image X and
                                                    %% associated colormap Map. The functions getframe and im2frame create a movie frame. 
                                                    %% If the frame contains true-color data, then Map is empty.
                                                    %% frame2im将单独的视频帧转换成索引图像。这个函数得到图像帧，而im2frame函数则产生一个视频帧。
                                                    %% 如果图像包括真彩数据，那么地Map就是空的了。
%     imwrite(x,'fln.jpg','jpg');                   %% 将得到的图像写到硬盘。
% end                                               %% 结束
    pixel(:,:,f) = (rgb2gray(pixels(:,:,:,f)));     % I = rgb2gray(RGB) converts the truecolor image RGB to the grayscale intensity image I.
                                                    % rgb2gray函数将真彩图片RGB转化为灰度图像I
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    标定图像    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rows=240;                                           % 定义标定中心点行号为240
cols=320;                                           % 定义标定中心点列号为320
nrames=f;                                           % 定义nrames为f

for l = 2:nrames                                    % 置图像处理循环变量l，从第二针图像开始处理.
                                                    %为图像标定的第一层循环.
d(:,:,l)=(abs(pixel(:,:,l)-pixel(:,:,l-1)));        % 图像相邻像素值之差取绝对值，并将其转存为矩阵d(:,:,l).？？？？

k=d(:,:,l);                                         % 定义变量k为像素差值矩阵,易于程序编写.
% imagesc(k);                                       %% imagesc:Scale data and display image object 标定数据并且显示图像对象
% drawnow;                                          %% 清除事件队列和更新数字窗口
% himage = imshow('d(:,:,l)');jii                   %% 显示图像储存在图形文件的文件名
% hfigure = figure;                                 %% 创建数字图形对象
% impixelregionpanel(hfigure, himage);              %% hpanel = impixelregionpanel(hparent,himage) creates a Pixel Region tool  panel associated with the image specified by the handle himage,
                                                    %% called the target image. 
                                                    %% 创建像素区域工具面板与图像处理指定所谓的目标图像。

% datar=imageinfo(imagesc(d(:,:,l)));               %% imagesc:Scale data and display image object 标定数据并且显示图像对象
                                                    %% imageinfo: creates an Image Information tool associated with the image in the current
                                                    %% figure.
                                                    %% 在当前的窗口中创建一个图像信息工具.
% disp(datar);                                      %% displays an array, without printing the array name. 
                                                    %% 显示一个数组同时缺省数组名称.


   bw(:,:,l) = im2bw(k, .2);                        % 灰度图像转换为二值图像,存为数组bw(:,:,l)。
   
   bw1=bwlabel(bw(:,:,l));                          % L = bwlabel(BW,n) returns a matrix L,of the same size as BW, containing labels for the connected objects in BW. 
                                                    % n can have a value of either 4 or 8, where 4 specifies 4-connected objects and 8 specifies 8-connected objects;
                                                    % if the argument is omitted, it defaults to 8.
                                                    % 返回一个与BW相同维数的矩阵L，其包含BW中连接对象的标签。 
                                                    % n可以有一个4或8
                                                    % ，其中指定4个相关对象或8个相关;如果参数缺省，则默认值为8 
                                                    % ？？？？？？？？不太清楚此步的作用。
   imshow(bw(:,:,l))                                % displays the binary image BW. imshow displays pixels with the value 0 (zero) as black and pixels with
                                                    % the value 1 as white.
                                                    % 显示二值图像BW。
                                                    % imshow显示器0值像素为黑色和1值像素为白色.
   hold on                                          % hold on :retains the current plot and certain axes properties so that subsequent graphing commands add to the
                                                    % existing graph.
                                                    % 保留目前的进程和某些轴特性，使下面的绘图命令应用到现有的图形当中.
   
% for h=1:rows                                      %% h--行循环变量
%     for w=1:cols                                  %% w--列循环变量
%                         
%             if(d(:,:,l)< 0.1)                     %% 判断图像相邻像素值之差取绝对值是否小于0.1
%                  d(h,w,l)=0;                      %% 是,则对应点像素值置0.否则保持.
                                                    %% 标定算法.
%             end
%      end
%   
% end
   
% %  disp(d(:,:,l));                                %% 显示差值标示
% %  size(d(:,:,l))                                 %% 显示差值大小
cou=1;                                              % 定义变量cou=1.
for h=1:rows                                        % 行循环起始值(步长为缺省值1)
    for w=1:cols                                    % 列循环起始值(步长为缺省值1)      %% 此处列循环为内循环,行循环为外循环.
     if(bw(h,w,l)>0.5)                              % 如果一点像素值bw(h,w,l)大于0.5
        
        
%          disp(d(h,w,l))；                         %% 显示变换后的图片数据帧
      toplen = h;                                   % 保存行值h,赋值给toplen.
        if (cou == 1)                               % 如果对应的列值为1.
            tpln=toplen;                            % 则转存toplen为tpln.
           
         end
         cou=cou+1;                                 % 判断下一行                              
        break                                       % 执行中止.(另有while循环功用)
     end                                            % if判断完成 
    end                                             % 列循环完成 
end                                                 % 行循环完成                                              

disp(toplen);                                       % 显示最首行行值

% % 下面循环结构与上面同,只是对像素值大于0.5的处理.
coun=1;                                             % 定义常值变量coun,为后面直接判断进入下一步程序所用.               
for w=1:cols                                        % 列循环变量h
    for h=1:rows                                    % 行循环变量h
     if(bw(h,w,l)>0.5)                              %如果一点像素值bw(h,w,l)大于0.5
        
      leftsi = w;                                   %转存列值w
      
    
   if (coun == 1)                                   % 若为同一个点则不进行操作.(此为)
            lftln=leftsi;                           % 且转存列值
            coun=coun+1;                            % 同时常量coun自加1,以跳出循环.
   end                                              % if判断结束
      break                                         % 跳出
     end                                            % 一个点判断完成
     
    end                                             % 一列判断完成 
end                                                 % 一行判断完成

disp(leftsi);                                       %disp(X) displays an array, without printing the array name. If X contains a text string, the string is displayed.
                                                    % 显示一个没有表示名称的数组。如果数组X包含一个文本字符串，则显示该字符串。
                                                    % 这里是显示最左侧起始列值
disp(lftln);                                        % 显示左侧首行行值


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  开始显示图像  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

 % % drawnow;                                       %% 图像显示开始                           
% %    d = abs(pixel(:, :, l), pixel(:, :, l-1));   % abs(X) returns an array Y such that each element of Y is the absolute value of the corresponding
                                                    % element of X.
                                                    %% 返回对应于每个元素x的绝对值的y值。
                                                    %% 这里将相邻行像素差值保存为矩阵d
% %    disp(d);                                     %% 显示矩阵d      
   
%    s = regionprops(bw1, 'BoundingBox');           %% STATS = regionprops(L, properties) measures a set of properties for each labeled region in the label matrix L.
                                                    %% 衡量（只是模糊分析）图像区域的性能.并将性能数据保存为变量矩阵
                                                    %% 此处及测定bwl区域的限定框特性,并将其保存为数组s
% %    centroids = cat(1, s.Centroid);              %% 将bwl区域的中心点数字矩阵转换为单一的矩阵centroids
% 
% %    ang=s.Orientation;                           %% Orientation表示:Convert viewpoint direction to VRML orientation将角度转向VRML的方向
                                                    %% 此为将转换后的特性数据保存为矩阵ang
%    
% %    plot(centroids(:,1), centroids(:,2), 'r*')   %% 使用图选择工作空间浏览器所选定的变量和图录。

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   此为生成对象外接矩形的注释   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    for r = 1 : length(s)                                     %% 处理图像区域的循环
%    rectangle('Position',s(r).BoundingBox,'EdgeColor','r');   %% rectangle draws a rectangle with Position [0,0,1,1] and Curvature [0,0] (i.e., no curvature).
                                                               %% rectangle以坐标[0,0,1,1]画长方形框并且曲率为0. 
                                                               
% %   plot('position',s(r).BoundingBox,'faceregion','r');      %% plot(...,'PropertyName',PropertyValue,...)  
                                                               %% sets properties to the specified property values for all lineseries graphics objects created by plot.
                                                               %% 给画图工具画出的连续图像的各个属性设定值。
%    end                                                       %% 设定参数结束
%    

% %    disp(ang);                                              %% 显示图像区域的性能数据矩阵                                            
%  %  imaqmontage(k);                                          %% 定义变量k为像素差值矩阵  
                                                               %% iimaqmontage(frames) displays a montage of image frames in a MATLAB figure window using the imagesc 
                                                               %% function.frames can be any data set returned by getdata, peekdata,or getsnapshot.
                                                               %% 该函数利用imagesc函数功能在figure窗口显示图像帧的细节.frames可以使任意getdata, peekdata,或 getsnapshot返回的数据集合

                                                               
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   显示相关图像大小参数  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

widh=leftsi-lftln;                                             % 标定框宽度
heig=toplen-tpln;                                              % 列值信息

widt=widh/2;                                                   % 确定标定中心的横（列）坐标.标定中心位于标定矩形框中心位置.
disp(widt);                                                    % 显示标定中心的列坐标.
heit=heig/2;                                                   % 确定标定中心的横（列）坐标.
with=lftln+widt;                                               % 赋值with=leftsi
heth=tpln+heit;                                                % 赋值heth=toplen
wth(l)=with;                                                   % 赋值wth(1)=with
hth(l)=heth;                                                   % 赋值hth(1)=heth

disp(heit);                                                    % 显示heit
disp(widh);                                                    % 显示widh
disp(heig);                                                    % 显示heig
rectangle('Position',[lftln tpln widh heig],'EdgeColor','r');  % rectangle(...,'Curvature',[x,y]) specifies the curvature  of the rectangle sides,
                                                               % enabling it to vary from a rectangle to an ellipse. The horizontal curvature x is the fraction of width of
                                                               % the rectangle that is curved along the top and bottom edges. The vertical curvature y is the fraction of 
                                                               % the height of the rectangle that is curved along the left and right edges.
                                                               % 矩形(...,'曲率' ， [的x ， y ] ）：通过指定矩阵的双边曲率，可以使标定框产生由矩形到椭圆形变化.横向曲率x是具有沿着
                                                               % 顶部和底部弯曲的矩形宽度数值.垂直曲率Y是沿着左右边方向弯曲的矩形高度数值.
disp(with);                                                    % 显示宽度
disp(heth);                                                    % 显示高度

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    生成外接矩形   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
plot(with,heth, 'r*');                                         % plots all lines defined by Xn versus Yn pairs.画出由Xn与Yn确定的线.
drawnow;                                                       % drawnow: flushes the event queue and updates the figure window. 清除缓存中的事件队列和更新数字窗口.
hold off                                                       % hold off ：resets axes properties to their defaults before drawing new plots. hold off is the default.
                                                               % hold off ：在画新的图形前重置坐标轴标度为默认值.hold off即为缺省值.

end;                                                           % 逐帧图像标定处理结束.
% wh=square(abs(wth(2)-wth(nrames)));
% ht=square(abs(hth(2)-hth(nrames)));
% disp(wth(1
% distan=sqrt(wh+ht);
% 
% disp(distan);


