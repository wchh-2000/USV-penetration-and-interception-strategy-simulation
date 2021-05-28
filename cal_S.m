function S=cal_S(p,xl,xu,yl,yu)
%p:n*2 存n个圆心坐标 x y投点范围[xl,xu]
close all
r=100;
[n,~]=size(p);
f=cell(n,1);
for i=1:n
f{i}=@(x,y)(x-p(i,1)).^2+(y-p(i,2)).^2-r^2;
end
N=1e5;%投点个数
x=xl+(xu-xl)*rand(1,N);
y=yl+(yu-yl)*rand(1,N);
M=zeros(1,N);
for i=1:n
    M=M+[f{i}(x,y)<=0];%方程小于等于0代表在圆内 0 1向量
end
id=[M>=2];%至少在两个圆内
m=sum(id);
plot(x(id(1:1e5)),y(id(1:1e5)),'r.')%展示1e5个点
axis equal
S=(xu-xl)*(yu-yl)*m/N;