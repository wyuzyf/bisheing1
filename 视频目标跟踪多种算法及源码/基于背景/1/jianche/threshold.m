%º¯Êý£ºãÐÖµÑ¡Ôñ
function T=threshold(I)
Id=im2double(I);
T=0.5*(min(Id(:)))+max((Id(:)));
deltaT=0.01;
done=false;
while ~done
    g=Id>=T;
    Tnext=0.5*(mean(Id(g))+mean(Id(~g)));
    done=abs(T-Tnext)<deltaT;
    T=Tnext;
end

