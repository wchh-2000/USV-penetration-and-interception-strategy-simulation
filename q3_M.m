function r=q3_M(M)
global ddBmax dt vB vRc0 vRcmax
L=1e4;
%M=10000;

pR1_1n=[L,7*M/10];%�����ĵ�1��ʼλ������[x,y]
pR2_1n=[L,3*M/10];
pRc1n=[L+200,pR1_1n(2)];%�����佢��ʼλ�� �ں켯Ⱥ֮��200m 
pRc2n=[L+200,pR2_1n(2)];%�Ժ�Rc��������佢��R����켯Ⱥ����

pR1_2n = pRc1n;    %�ڶ����ͷ�ǰ�ڶ���λ�������佢һ��
pR2_2n = pRc2n;

pBn=[0,M*1/2];
Dwave2 = 6500;%�ͷŵڶ���x������ֵ����
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

dBn=0;%����ʼ�����
% dR1n=atan((pBn(2)-pR1n(2))/(pBn(1)-pR1n(1)))+pi;%��1��ʼ������ָ������
% dR2n=atan((pBn(2)-pR2n(2))/(pBn(1)-pR2n(1)))+pi;%��23���޽ǣ���x�������˶�
dR1_1n=pi; dR2_1n=pi;dR1_2n=pi; dR2_2n=pi;%�췽USV��ʼ����

dRc1n=cal_dRc_init(pRc1n,pBn); 
dRc2n=cal_dRc_init(pRc2n,pBn);%�췽���佢��ʼ����

Dth1 = 88;Dth2 = 159;%2�������ذ뾶

t=0;
vRc1=vRc0; vRc2=vRc0;
wave2 = [0 0];%��ʼ���ж��Ƿ��Ѿ����ɵڶ���
while pBn(1)<L & t<700 & min(norm(pBn-pR1_1n),norm(pBn-pR2_1n))>Dth1 &...
        min(norm(pBn-pR1_2n),norm(pBn-pR2_2n))>Dth2
    %����λ��
    pB=pBn;
    pR1_1=pR1_1n; pR2_1=pR2_1n; pRc1=pRc1n; pRc2=pRc2n;
    pR1_2=pR1_2n; pR2_2=pR2_2n;%�ڶ���
    
    %���·���
    dB=dBn;
    dR1_1=dR1_1n; dR2_1=dR2_1n; dRc1=dRc1n; dRc2=dRc2n;
    dR1_2=dR1_2n; dR2_2=dR2_2n;
    
    %�ж��ͷŵڶ���
    while wave2(1)==0 & pB(1)>pR1_1(1)-Dwave2  
        %����USV��ͻ�Ƶ�1�����佢�ĵ�һ����,���ɵڶ�����
        if pB(2)>=pRc1+200   %Y�������������佢������������Ϸ�
            pR1_2 = [pRc1(1) pRc1(2)+200];%�������Ϸų�
        else
            if pB(2)<pRc1+200 & pB(2)>pRc1-200
                pR1_2 = [pRc1(1) pR(2)];
            else
                pR1_2 = [pRc1(1) pRc1(2)-200];
            end
        end
        wave2(1) = 1;
    end
    while wave2(2)==0 & pB(1)>pR2_1(1)-Dwave2  
        %����USVͻ�Ƶ�2�����佢�ĵ�һ����,���ɵڶ�����
        if pB(2)>=pRc2+200
            pR2_2 = [pRc2(1) pRc2(2)+200];
        else
            if pB(2)<pRc2+200 & pB(2)>pRc2-200
                pR2_2 = [pRc2(1) pR(2)];
            else
                pR2_2 = [pRc2(1) pRc2(2)-200];
            end
        end
        wave2(2) = 1;
    end
    
    dBn=cal_direcB(dB,pB,pR1_1,pR2_1,pR1_2,pR2_2,M);%��һ���ٶȷ����
    pBn=cal_position(pB,vB,dB);%��һ��λ������
    
    %������λ������������ٶȷ��� 
    %������һ����ķ���ǣ�����λ������ΪĿ�꣬���λ�����꣬�����һ����ķ����
%     dR1_1n=cal_direcR_1(dR1_1,pB,pR1_1,ddRmax,50,wave2(1)); %��һ��1_1�ٶȷ����
%     dR2_1n=cal_direcR_1(dR2_1,pB,pR2_1,ddRmax,50,wave2(2)); %��һ��2_1�ٶȷ����
    dR1_1n=cal_direc(dR1_1,pB,pR1_1,ddRmax); %��һ��1_1�ٶȷ����
    dR2_1n=cal_direc(dR2_1,pB,pR2_1,ddRmax); %��һ��2_1�ٶȷ����
    if wave2(1)==1
        dR1_2n=cal_direcR(dR1_2,pB,pR1_2,ddRmax,50);%��һ��1_2�ٶȷ����
        pR1_2n=cal_position(pR1_2,vR,dR1_2);%���ٶȷ���ȷ����һ��1_2λ������
    end
    if wave2(2)==1
        dR2_2n=cal_direcR(dR2_2,pB,pR2_2,ddRmax,50);%��һ��2_2�ٶȷ����
        pR2_2n=cal_position(pR2_2,vR,dR2_2);%���ٶȷ���ȷ����һ��2_2λ������
    end
    
    ddRcmax1=dt*vRc1/rRc;
    ddRcmax2=dt*vRc2/rRc;
    dRc1n=cal_direc(dRc1,(pR1_1+pR1_2)/2,pRc1,ddRcmax1);
    dRc2n=cal_direc(dRc2,(pR2_1+pR2_2)/2,pRc2,ddRcmax2);
    
    pR1_1n=cal_position(pR1_1,vR,dR1_1);%���ٶȷ���ȷ����һ��1λ������
    pR2_1n=cal_position(pR2_1,vR,dR2_1);
    
    
    vRc1=get_vRc(pR1_1,pB,pRc1);%������佢�ٶȴ�С �����λ�þ���
    vRc2=get_vRc(pR2_1,pB,pRc2);         %�ڶ��������ݲ�����
    
    pRc1n=cal_position(pRc1,vRc1,dRc1);%��һ�����ؽ�1λ������
    pRc2n=cal_position(pRc2,vRc2,dRc2);
    if min(norm(pRc1n-pB),norm(pRc2n-pB))<1000 
        %disp('��������ؽ��������')
        r=2;
        return
    end
    if max(norm(pRc1n-pR1_1n),norm(pRc2n-pR2_1n))>2000+Dth1
        %disp('�����ؽ��뼯Ⱥ�����Զ')
        r=3;
        return
    end
%     if mod(t,5)<0.1 %5s����һ��ͼ
%         scatter(pBn(1),pBn(2),'b.')
%         scatter(pR1_1n(1),pR1_1n(2),'r')
%         scatter(pR2_1n(1),pR2_1n(2),'r')
%         scatter(pRc1n(1),pRc1n(2),'g')
%         scatter(pRc2n(1),pRc2n(2),'g')
%         if wave2(1)==1 
%             scatter(pR1_2n(1),pR1_2n(2),'r')
%         end
%         if wave2(2)==1 
%             scatter(pR2_2n(1),pR2_2n(2),'r')
%         end
%     end
%     pause(0.002)
    t=t+dt;
end
if min(norm(pBn-pR1_1n),norm(pBn-pR2_1n))<=Dth1 || min(norm(pBn-pR1_2n),norm(pBn-pR2_2n))<=Dth2
    %disp('successful interception')
    r=0;
elseif t>=700
    %disp('time out')
    r=0;
elseif pBn(1)>=L
    %disp('successful penetration')
    r=1;
else 
r=-1;
end

function dBn=cal_direcB(dB,pB,pR1,pR2,pR3,pR4,M)
%������һ�����ķ���ǣ�����λ�����꣬���λ�����꣬�����һ�����ķ����
global ddBmax vB
Dbr=min([norm(pR1-pB),norm(pR2-pB),norm(pR3-pB),norm(pR4-pB)]);
D0=600;%�ٽ���� �þ�����������ֱ�ٶ����
Deth=500;%������߽������ֵ��С�ں���ֱ�ٶȼ�С
a0=10/M;
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

function pn=cal_position(p,v,d)
%�ɵ�ǰλ�á��ٶȴ�С���ٶȷ���Ǽ�����һ��λ������pn
%΢Ԫ�� ÿС�ν���Ϊֱ���˶�
global dt
vx=v*cos(d);
vy=v*sin(d);
xn=p(1)+vx*dt;
yn=p(2)+vy*dt;
pn=[xn yn];

