% demonstrate use of hhist

figure;
x = sample([0.1 0.2 0.3 0.4], 1000);
hhist(x, linspace(0,5,100));

figure;
% verify density for product of indep normals
x = prod(randn(2,100000));
ts = -1:0.01:1;
z = hhist(x, ts);
plot(ts, z, ts, besselk(0,abs(ts))*1/pi)    
