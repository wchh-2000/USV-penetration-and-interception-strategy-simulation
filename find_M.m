function find_M()
close all
M=100:100:25000;
n=length(M);
r=zeros(1,n);
for i=1:n
r(i)=q3_M(M(i));
if mod(i,10)==0
disp(i)
end
end
stem(M,r)
xlabel('M/m')
set(gca,'ytick',[0 1 2 3])
set(gca,'yticklabel',{'intercept','penetrate','br too close','rRc too far'}) 
set(gca,'FontSize',14) 