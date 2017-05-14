function img2eps(imgname, epsname)
% img2eps(imgname, espname) 	: convert bmp, jpg, gif file into eps file.
% imgname: the file name of the image
% epsname: the eps file name
% Copyright@Jilin Tu, IFP group, UIUC, 2003. 

[img, map]=imread(imgname);
figure(100);
imagesc(img);colormap(map);axis image; axis off;
print('-depsc2', epsname);
close(100);