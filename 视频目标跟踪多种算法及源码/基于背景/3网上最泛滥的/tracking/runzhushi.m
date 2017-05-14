clear data                                       % clear keyword :clears the items indicated by keyword.
                                                 % 清除由关键词 keyword 所表示的项目.
disp('input video');                             % 显示说明字符串:input video.                            


avi = aviread('zipai.avi');                      % mov = aviread(filename) reads the AVI movie filename into the MATLAB movie structure mov.                      
                                                 % 读取一段AVI电视频文件到MATLAB视频缓存中. 
                                                 % 并将avi视频文件信息保存为矩阵avi.
                                                 
video = {avi.cdata};                             % cdata 的含义见网页:http://www.baisi.net/thread-6612-1-1.html
for a = 1:length(video)                        % 逐帧处理数据    
    imagesc(video{a});                           % imagesc(C) displays C as an image. Each element of C corresponds to a rectangular area in the image.
                                                 % The values of the elements of C are indices into the current colormap that determine the color of each patch. 
                                                 % imagesc(c):将矩阵C显示为图像格式.C
                                                 % 的每一个元素对应于图像中的一个矩形区域.C中元素
                                                 % 都被写入当前决定每个补丁颜色的伪彩色图中.
                                             
                                                 % colormap--伪彩色图，显示强度在平面的分布（类似等值线图，或地图上的等深线图等），伴随它应有色彩标尺

    axis image off                               % axis off :turns off all axis lines,tick marks, and labels.
                                                 % 关闭所有的坐标轴、标签和二维图像标签.
    drawnow;                                     % drawnow: flushes the event queue and updates the figure window. 清除缓存中的事件队列和更新数字窗口.
end;                                             % 图像帧提取完成.            
disp('output video');                            % 显示字符串"output video"
tracking(video);                                 % 调用函数tracking.
