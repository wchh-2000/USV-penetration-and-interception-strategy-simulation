function [S,Min,Ml]=find_fastest_SA()
global n T0 Range%ȫ�ֱ���
T0=50; %��ʼ���¶�ֵ 
T_min=0.1; %�����¶��½� 
r=0.98; %�¶ȵ��½��� 
n=4;%���ά��
Range=[130,400;
    500,3000;
    -400,100;
    0.0001,0.002];%��ά������
k=20; %ͬһ�¶���ѭ������  
S=zeros(1,n);
Min=0;
for i=1:n
S(i)=(Range(i,2)-Range(i,1))*rand()+Range(i,1); %����õ���ʼ��range(i,1)~range(i,2)
end
c=0;
cp=0;
Ml=[];%��¼��Сֵ�½�
T=T0;
hold on
while(T>T_min)     
    for i=1:k
    f=q2_t(S);
    S_new=getS(S,T); %���ݵ�ǰ��͵�ǰ�¶Ȳ����½�
    f_new=q2_t(S_new);%�µĺ���ֵ
    delta=f_new-f;
    if (delta<0) %�½����
        S=S_new;
        Min=f_new;
    elseif exp(-delta/T)>rand %�½�ϲ�ʱ��һ�����ʽ����½�
        S=S_new;
        Min=f_new;
    end
    end
T=T*r;  %����
if mod(c,10)<0.1
    cp=cp+1;
    fprintf('%f:%f\n',T,Min)
    Ml=[Ml Min];
    plot(cp,Min,'o');
end
c=c+1;%���ڼ�¼ѭ����������ʾ�������
end
%fprintf('f(%.4f, %.4f)=%.4f\n',S(1),S(2),fun3(S))
% fprintf('fun3���Ž�Ϊ:x=%f, y=%f\n',S(1),S(2)) 
% fprintf('��СֵΪ��%f\n',fun3(S)) 
% disp('fun1���Ž�Ϊ:') 
% disp(S)
% fprintf('��СֵΪ��%f\n',fun1(S)) 


function S_new=getS(S,T) %�����½�
global n T0 Range
S_new=zeros(1,n);
for i=1:n
    u=Range(i,2);
    l=Range(i,1);
    S_new(i)=S(i)+(u-l)/2*T/T0*(2*rand()-1); 
    %��ԭ������ϼ��Ϸ�Χ��T�仯���������TԽС���½�Խ����
    if S_new(i)>u
        S_new(i)=u;
    elseif S_new(i)<l
        S_new(i)=l;
    end
end



