function S=cal_S(p,xl,xu,yl,yu)
%p:n*2 ��n��Բ������ x yͶ�㷶Χ[xl,xu]
close all
r=100;
[n,~]=size(p);
f=cell(n,1);
for i=1:n
f{i}=@(x,y)(x-p(i,1)).^2+(y-p(i,2)).^2-r^2;
end
N=1e5;%Ͷ�����
x=xl+(xu-xl)*rand(1,N);
y=yl+(yu-yl)*rand(1,N);
M=zeros(1,N);
for i=1:n
    M=M+[f{i}(x,y)<=0];%����С�ڵ���0������Բ�� 0 1����
end
id=[M>=2];%����������Բ��
m=sum(id);
plot(x(id(1:1e5)),y(id(1:1e5)),'r.')%չʾ1e5����
axis equal
S=(xu-xl)*(yu-yl)*m/N;