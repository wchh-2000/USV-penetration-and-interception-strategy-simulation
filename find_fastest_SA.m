function [S,Min,Ml]=find_fastest_SA()
global n T0 Range%全局变量
T0=50; %初始化温度值 
T_min=0.1; %设置温度下界 
r=0.98; %温度的下降率 
n=4;%解的维数
Range=[130,400;
    500,3000;
    -400,100;
    0.0001,0.002];%各维定义域
k=20; %同一温度下循环次数  
S=zeros(1,n);
Min=0;
for i=1:n
S(i)=(Range(i,2)-Range(i,1))*rand()+Range(i,1); %随机得到初始解range(i,1)~range(i,2)
end
c=0;
cp=0;
Ml=[];%记录最小值下降
T=T0;
hold on
while(T>T_min)     
    for i=1:k
    f=q2_t(S);
    S_new=getS(S,T); %根据当前解和当前温度产生新解
    f_new=q2_t(S_new);%新的函数值
    delta=f_new-f;
    if (delta<0) %新解更优
        S=S_new;
        Min=f_new;
    elseif exp(-delta/T)>rand %新解较差时以一定概率接受新解
        S=S_new;
        Min=f_new;
    end
    end
T=T*r;  %降温
if mod(c,10)<0.1
    cp=cp+1;
    fprintf('%f:%f\n',T,Min)
    Ml=[Ml Min];
    plot(cp,Min,'o');
end
c=c+1;%用于记录循环次数，显示程序进度
end
%fprintf('f(%.4f, %.4f)=%.4f\n',S(1),S(2),fun3(S))
% fprintf('fun3最优解为:x=%f, y=%f\n',S(1),S(2)) 
% fprintf('最小值为：%f\n',fun3(S)) 
% disp('fun1最优解为:') 
% disp(S)
% fprintf('最小值为：%f\n',fun1(S)) 


function S_new=getS(S,T) %产生新解
global n T0 Range
S_new=zeros(1,n);
for i=1:n
    u=Range(i,2);
    l=Range(i,1);
    S_new(i)=S(i)+(u-l)/2*T/T0*(2*rand()-1); 
    %在原解基础上加上范围随T变化的随机量，T越小，新解越集中
    if S_new(i)>u
        S_new(i)=u;
    elseif S_new(i)<l
        S_new(i)=l;
    end
end



