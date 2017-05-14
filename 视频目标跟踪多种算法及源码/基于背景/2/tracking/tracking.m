function d = tracking(video,rows,cols)
if ischar(video)
    % Load the video from an avi file.
    avi = aviread(video);
    pixels = double(cat(4,avi(1:2:end).cdata))/255;
    clear avi
else
    % Compile the pixel data into a single array
    pixels = double(cat(4,video{1:2:end}))/255;
    clear video
end

% Convert to RGB to GRAY SCALE image.
nFrames = size(pixels,4);
for f = 1:nFrames
    r=pixels(:,:,1,f);
    g=pixels(:,:,2,f);
    b=pixels(:,:,3,f);
    R=histeq(r);
    G=histeq(g);
    B=histeq(b);
    result_rgb=cat(3,R,G,B);
    pixel(:,:,f) = (rgb2gray(result_rgb));
end
nrames=f;
for l = 2:nrames
    
    d(:,:,l)=(abs(pixel(:,:,l)-pixel(:,:,l-1)));
    
    k=d(:,:,l);
    
    bw(:,:,l) = im2bw(k, .2);
    
    bw1=bwlabel(bw(:,:,l));
    imshow(bw(:,:,l))
    hold on
    
    cou=1;
    for h=1:rows
        for w=1:cols
            if(bw(h,w,l)>0.5)
                
                toplen = h;
                
                if (cou == 1)
                    tpln=toplen;
                    
                end
                cou=cou+1;
                break
            end
            
        end
    end
    
    disp(toplen);
    
    coun=1;
    for w=1:cols
        for h=1:rows
            if(bw(h,w,l)>0.5)
                
                leftsi = w;
                
                
                if (coun == 1)
                    lftln=leftsi;
                    coun=coun+1;
                end
                break
            end
            
        end
    end
    
    disp(leftsi);
    disp(lftln);
    
    widh=leftsi-lftln;
    heig=toplen-tpln;
    
    widt=widh/2;
    disp(widt);
    heit=heig/2;
    with=lftln+widt;
    heth=tpln+heit;
    wth(l)=with;
    hth(l)=heth;
    
    disp(heit);
    disp(widh);
    disp(heig);
    rectangle('Position',[lftln tpln widh heig],'EdgeColor','r');
    disp(with);
    disp(heth);
    plot(with,heth, 'r*');
    drawnow;
    hold off
    
end;