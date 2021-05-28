function r=q3_1(M)%�������
global ddBmax dt vB L vRc0 vRcmax
L=1e4;
%M=10000;
% close all
% hold on
% axis([0 L+200 0 M])%�����᷶Χ
%��ʼλ�ã��췽һ�����ͷ�
pRc1n = [L,7.5*M/10];
pRc2n = [L,2.5*M/10];
pR1n = [L,7.5*M/10 + 200];
pR2n = [L,7.5*M/10 - 200];%12��Ӧ���佢1
pR3n = [L,2.5*M/10 + 200];
pR4n = [L,2.5*M/10 - 200];%34��Ӧ���佢2
pBn = [0,M*1/2];
pD1n = [L/4 4*M/5];    %��ʼ�����������
pD2n = [L/4 3*M/5];
pD3n = [L/4 2*M/5];
pD4n = [L/4 1*M/5];
pD10 = [L*2/3 9*M/10];    %��ʼ�����������
pD20 = [L/2 3*M/5];
pD30 = [L/2 2*M/5];
pD40 = [L*2/3 1*M/10];

rB=100;%����Сת��뾶
rR=80;
rRc=500;
vB=25;%���ٶ�m/s
vR=20;%�����ĵ��ٶ�m/s
vRc0=0;%�����佢��ʼ�ٶ�
vRcmax=16;%�����佢����ٶ�
ddBmax=dt*vB/rB;%������仯���Ƕ� 
ddRmax=dt*vR/rR;
dt=1;%ģ����ʱ��

%��ʼ�����
dBn=0;
% dR1n=atan((pBn(2)-pR1n(2))/(pBn(1)-pR1n(1)))+pi;%��1��ʼ������ָ������
% dR2n=atan((pBn(2)-pR2n(2))/(pBn(1)-pR2n(1)))+pi;%��23���޽ǣ���x�������˶�
dR1n=pi; dR2n=pi;dR3n=pi; dR4n=pi;%���к췽USV��ʼ����

dRc1n=cal_dRc_init(pRc1n,pBn); 
dRc2n=cal_dRc_init(pRc2n,pBn);%�췽���佢��ʼ����

Dth = 123.5;%���ذ뾶
D_attack = 1000;  %������������һ����������ܽ�ʱ���巢�𹥻�

t=0;
vRc1=vRc0; vRc2=vRc0;
attack = 0;%��ʼ��״̬��0Ϊδ�����ܹ���1Ϊ�����ܹ���
while pBn(1)<L & t<700 & ...
    min([norm(pBn-pR1n),norm(pBn-pR2n),norm(pBn-pR3n),norm(pBn-pR4n)])>Dth 
       
    %����λ��
    pB=pBn;
    pR1=pR1n; pR2=pR2n;  pR3=pR3n; pR4=pR4n;
    pRc1=pRc1n; pRc2=pRc2n;
%     pD1=pD1n;pD2=pD2n;pD3=pD3n;pD4=pD4n;
    
%     if min([norm(pBn-pD1n),norm(pBn-pD2n),norm(pBn-pD3n),norm(pBn-pD4n)])<D_attack
    if min([norm(pBn-pD10),norm(pBn-pD20),norm(pBn-pD30),norm(pBn-pD40)])<D_attack
        attack = 1;
    end
    %���·���
    dB=dBn;
    dR1=dR1n; dR2=dR2n; dR3=dR3n; dR4=dR4n;
    dRc1=dRc1n; dRc2=dRc2n;
    
    dBn=cal_direcB(dB,pB,pR1,pR2,pR3,pR4,M);%��һ���ٶȷ����
    pBn=cal_position(pB,vB,dB);%��һ��λ������
    
    %��������㷨�������ٶȷ���
    %������һ����ķ���ǣ�Ŀ��Ϊ������أ������һ����ķ����
    if attack == 0      %2�ֽ׶κ췽USV��Ŀ��
        dR1n=cal_direc(dR1,pD10,pR1,ddRmax);   %��һ��1�ٶȷ����
        dR2n=cal_direc(dR2,pD20,pR2,ddRmax);   %��һ��2�ٶȷ����
        dR3n=cal_direc(dR3,pD30,pR3,ddRmax);   %��һ��3�ٶȷ����
        dR4n=cal_direc(dR4,pD40,pR4,ddRmax);   %��һ��4�ٶȷ����
    else
        dR1n=cal_direc(dR1,pB,pR1,ddRmax);   %��һ��1�ٶȷ����
        dR2n=cal_direc(dR2,pB,pR2,ddRmax);   %��һ��2�ٶȷ����
        dR3n=cal_direc(dR3,pB,pR3,ddRmax);   %��һ��3�ٶȷ����
        dR4n=cal_direc(dR4,pB,pR4,ddRmax);   %��һ��4�ٶȷ����
    end
    
    pR1n=cal_position(pR1,vR,dR1);%���ٶȷ���ȷ����һ��1λ������
    pR2n=cal_position(pR2,vR,dR2);%���ٶȷ���ȷ����һ��2λ������
    pR3n=cal_position(pR3,vR,dR3);%���ٶȷ���ȷ����һ��3λ������
    pR4n=cal_position(pR4,vR,dR4);%���ٶȷ���ȷ����һ��4λ������
%     pD1n = cal_pD(pD1,pB);          %ȷ����һʱ��Ŀ�����λ������
%     pD2n = cal_pD(pD2,pB);
%     pD3n = cal_pD(pD3,pB);
%     pD4n = cal_pD(pD4,pB);
    
    ddRcmax1=dt*vRc1/rRc;
    ddRcmax2=dt*vRc2/rRc;
    dRc1n=cal_direc(dRc1,(pR1+3*pR2)/4,pRc1,ddRcmax1);%��һ�����佢1�ٶȷ����
    dRc2n=cal_direc(dRc2,(pR4+3*pR3)/4,pRc2,ddRcmax2);%��һ�����佢2�ٶȷ����
    
   
    vRc1=get_vRc(pR1,pB,pRc1);%������佢�ٶȴ�С �����λ�þ���
    vRc2=get_vRc(pR2,pB,pRc2);         %�ڶ��������ݲ�����
    
    pRc1n=cal_position(pRc1,vRc1,dRc1);%��һ�����ؽ�1λ������
    pRc2n=cal_position(pRc2,vRc2,dRc2);%��һ�����ؽ�2λ������
    
    if min(norm(pRc1n-pB),norm(pRc2n-pB))<1000 
        %disp('��������ؽ��������')
        r=2;
        return
    end
    if max([norm(pRc1n-pR1n),norm(pRc1n-pR2n),norm(pRc2n-pR3n),norm(pRc2n-pR4n)])>2000+Dth
        %disp('�����ؽ��뼯Ⱥ�����Զ')
        r=3;
        return
    end
%     if mod(t,5)<0.1 %5s����һ��ͼ
%         scatter(pBn(1),pBn(2),'b.')
%         scatter(pR1n(1),pR1n(2),'r')
%         scatter(pR2n(1),pR2n(2),'r')
%         scatter(pR3n(1),pR3n(2),'r')
%         scatter(pR4n(1),pR4n(2),'r')
%         scatter(pRc1n(1),pRc1n(2),'g')
%         scatter(pRc2n(1),pRc2n(2),'g')
%         plot(pD10(1),pD10(2),'m*')
%         plot(pD20(1),pD20(2),'m*')
%         plot(pD30(1),pD30(2),'m*')
%         plot(pD40(1),pD40(2),'m*')
%     end
%     pause(0.002)
    t=t+dt;
end
if min([norm(pBn-pR1n),norm(pBn-pR2n),norm(pBn-pR3n),norm(pBn-pR4n)])<=Dth 
    %disp('successful interception')
    r=0;
elseif t>=700
    %disp('time out')
    r=0;
elseif pBn(1)>=L
    %disp('successful penetration')
    r=1;
end
end

function dBn=cal_direcB(dB,pB,pR1,pR2,pR3,pR4,M)
%������һ�����ķ���ǣ�����λ�����꣬���λ�����꣬�����һ�����ķ����
global ddBmax vB
Dbr=min([norm(pR1-pB),norm(pR2-pB),norm(pR3-pB),norm(pR4-pB)]);
D0=300;%�ٽ���� �þ�����������ֱ�ٶ����
Deth=500;%������߽������ֵ��С�ں���ֱ�ٶȼ�С
a0=0.001;
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
    vy=vy*(-1/Deth^2*De*(De-2*Deth));
end
vx=sqrt(vB^2-vy^2);
dBn=atan(vy/vx);
if abs(dBn-dB)>ddBmax
    %disp(1)
    if dBn>dB
        dBn=dB+ddBmax;
    else
        dBn=dB-ddBmax;
    end
end
end
function v=get_vRc(pR,pB,pRc)
global vRc0 vRcmax
Dr=norm(pRc-pR);
v=(vRcmax-vRc0)./(1+exp(-Dr/100+5))+vRc0;
Db=norm(pRc-pB);
Dbth=3000;
%��B�����ж�
if Db<Dbth
    k=-1/(Dbth-1000)^2*(Db-1000)*(Db+1000-2*Dbth);
else
    k=1;
end
v=v*k;
end
function pn=cal_position(p,v,d)
%�ɵ�ǰλ�á��ٶȴ�С���ٶȷ���Ǽ�����һ��λ������pn
%΢Ԫ�� ÿС�ν���Ϊֱ���˶�
global dt
vx=v*cos(d);
vy=v*sin(d);
xn=p(1)+vx*dt;
yn=p(2)+vy*dt;
pn=[xn yn];
end
function pDn = cal_pD(pD,pB)
global L
%�ɵ�ǰ���λ�ú���USVλ�ñ仯����һ�����λ�ã��ƶ����Ǿ���ĺ���
d_DB = norm(pD-pB);
bP = L*1/4;         %����λ��ƽ�ƺ���Ϊֱ�ߣ������ʾ�ؾ��б��
kP = -bP/7500;
Dx = kP*d_DB+bP;
pDn = [Dx+pD(1),pD(2)];
end