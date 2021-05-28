function find_init
close all
L=1e4;
M=7000;
N=1000;
% x=6000+4000*rand(1,N);
% y=1000+5000*rand(1,N);
x=10000*rand(1,N);
y=7000*rand(1,N);
c=0;
fail=[];
Close=[];
pene=[];
far=[];
for i=1:N
    r=q1([x(i) y(i)]);
    if r==2
       Close=[Close;[x(i) y(i)]];
       r=r/2;
    elseif r==3
        far=[far;[x(i) y(i)]];
        r=r/3;
    elseif r==0
       fail=[fail;[x(i) y(i)]];
    else
        pene=[pene;[x(i) y(i)]];
    end
   c=c+r;
end
hold on
axis([0 10000 0 7000])
plot(pene(:,1),pene(:,2),'k+')
plot(fail(:,1),fail(:,2),'ro',Close(:,1),Close(:,2),'bo',far(:,1),far(:,2),'go')
legend('penetrate','intercept','B R too close','R Rc too far')
set(gca,'FontSize',11) 
disp('penetration rate:')
disp(c/N)