function t=q2_t(X)
%X:[390,2000,0,0.001]
D0=X(1);
Deth=X(2); 
Dx=X(3);
a0=X(4);
global ddBmax dt vB M vRc0 vRcmax 
L=1e4;
M=7000;
close all
hold on
axis([0 L+200 0 M])%�����᷶Χ
% set(0,'defaultfigurecolor','w')
pR1n=[L,5*M/7];%�����ĵ�1��ʼλ������[x,y]
pR2n=[L,2*M/7];
pRc1n=[L+200,pR1n(2)];%�����佢��ʼλ�� �ں켯Ⱥ֮��200m 
pRc2n=[L+200,pR2n(2)];%�Ժ�Rc��������佢��R����켯Ⱥ����
pBn=[0,M/3];
rB=100;%����Сת��뾶
rR=80;
rRc=500;
vB=25;%���ٶ�m/s
vR=20;%�����ĵ��ٶ�m/s
vRc0=0;%�����佢��ʼ�ٶ�
vRcmax=16;%�����佢����ٶ�
ddBmax=dt*vB/rB;%������仯���Ƕ� 
ddRmax=dt*vR/rR;
dt=1;%ģ����ʱ��/s
dBn=0;%����ʼ�����
dR1n=atan((pBn(2)-pR1n(2))/(pBn(1)-pR1n(1)))+pi;%��1��ʼ������ָ������
dR2n=atan((pBn(2)-pR2n(2))/(pBn(1)-pR2n(1)))+pi;%��23���޽ǣ���x�������˶�
dRc1n=pi; dRc2n=pi;
Dth=123;
t=0;
d=0;%���ڱ����������ؽ�������������
r=0;%���ڱ�Ǻ����ؽ��뼯Ⱥ�����Զ
vRc1=vRc0; vRc2=vRc0;
while pBn(1)<L & t<700 & min(norm(pBn-pR1n),norm(pBn-pR2n))>Dth
    pB=pBn;%����λ��
    pR1=pR1n; pR2=pR2n; pRc1=pRc1n; pRc2=pRc2n;
    dB=dBn;%���·���
    dR1=dR1n; dR2=dR2n; dRc1=dRc1n; dRc2=dRc2n;
    
    dBn=cal_direcB(dB,pB,pR1,pR2,D0,Deth,Dx,a0);%��һ���ٶȷ����
    pBn=cal_position(pB,vB,dB);%��һ��λ������
    
    %������λ������������ٶȷ���
    dR1n=cal_direc(dR1,pB,pR1,ddRmax);%��һ��1�ٶȷ����
    %������һ����ķ���ǣ�����λ������ΪĿ�꣬���λ�����꣬�����һ����ķ����
    dR2n=cal_direc(dR2,pB,pR2,ddRmax);
    
    ddRcmax1=dt*vRc1/rRc;
    ddRcmax2=dt*vRc2/rRc;
    dRc1n=cal_direcRc(dRc1,pB,pR1,pRc1,ddRcmax1);%��һ�����佢1�ٶȷ����
    %�켯Ⱥ��λ������ΪĿ��
    dRc2n=cal_direcRc(dRc2,pB,pR2,pRc2,ddRcmax2);
    
    pR1n=cal_position(pR1,vR,dR1);%���ٶȷ���ȷ����һ��1λ������
    pR2n=cal_position(pR2,vR,dR2);
    vRc1=get_vRc(pR1,pB,pRc1);%������佢�ٶȴ�С �����λ�þ���
    vRc2=get_vRc(pR2,pB,pRc2);
    
    pRc1n=cal_position(pRc1,vRc1,dRc1);%��һ�����ؽ�1λ������
    pRc2n=cal_position(pRc2,vRc2,dRc2);
    if min(norm(pRc1n-pB),norm(pRc2n-pB))<1000 
        d=1;%��������ؽ��������
    end
    if max(norm(pRc1n-pR1n),norm(pRc2n-pR2n))>2000+Dth
        r=1;%�����ؽ��뼯Ⱥ�����Զ
    end
    if mod(t,5)<0.1 %5s����һ��ͼ
        scatter(pBn(1),pBn(2),'b.')
        scatter(pR1n(1),pR1n(2),'r')
        scatter(pR2n(1),pR2n(2),'r')
        scatter(pRc1n(1),pRc1n(2),'g')
        scatter(pRc2n(1),pRc2n(2),'g')
    end
    pause(0.0005)
    t=t+dt;
end
% if d==1
%     disp('��������ؽ��������')
% end
% if r==1
%     disp('�����ؽ��뼯Ⱥ�����Զ')
% end
if min(norm(pBn-pR1n),norm(pBn-pR2n))<=Dth
    %disp('successful interception')
    t=10000;%ʧ�� ��t�ϴ�ֵ
elseif t>=700
    disp('time out')
    t=10000;%ʧ�� ��t�ϴ�ֵ
% elseif pBn(1)>=L
%     
%     disp('successful penetration')
% else
%     disp('0')
end

function dBn=cal_direcB(dB,pB,pR1,pR2,D0,Deth,Dx,a0)
%������һ�����ķ���ǣ�����λ�����꣬���λ�����꣬�����һ�����ķ����
global ddBmax vB M
if min((pB(1)-pR1(1)),(pB(1)-pR2(1)))>Dx%ˮƽ�������ں��Ҳ�Dx
    dBn=0;%��ֱ��ˮƽ��������
else
    Dbr=min(norm(pR1-pB),norm(pR2-pB));
    if pB(2)<M/2 %�°�ƽ���������˶�
        k=-1;
    else
        k=1;
    end
    if Dbr>D0
        vy=k*vB*exp(-a0*(Dbr-D0));
    else
        vy=k*vB;
    end
    if k==1%�����˶�
        De=M-pB(2);%���ϱ߽�ľ���
    else%����
        De=pB(2);
    end
    if De<Deth
        vy=vy*(-3/Deth^2*De*(De-2*Deth)-2);
    end
    vx=sqrt(vB^2-vy^2);
    dBn=atan(vy/vx);
end
if abs(dBn-dB)>ddBmax%��Сת��뾶������
    %disp(1)
    if dBn>dB
        dBn=dB+ddBmax;
    else
        dBn=dB-ddBmax;
    end
end

function v=get_vRc(pR,pB,pRc)
global vRc0 vRcmax
Dr=norm(pRc-pR);
v=(vRcmax-vRc0)./(1+exp(-Dr/100+5))+vRc0;
Db=norm(pRc-pB);
Dbth=4000;
if Db<Dbth
    k=-1/(Dbth-1000)^2*(Db-1000)*(Db+1000-2*Dbth);
else
    k=1;
end
v=v*k;

function pn=cal_position(p,v,d)
%�ɵ�ǰλ�á��ٶȴ�С���ٶȷ���Ǽ�����һ��λ������pn
%΢Ԫ�� ÿС�ν���Ϊֱ���˶�
global dt
vx=v*cos(d);
vy=v*sin(d);
xn=p(1)+vx*dt;
yn=p(2)+vy*dt;
pn=[xn yn];

