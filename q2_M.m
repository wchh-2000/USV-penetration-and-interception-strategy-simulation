function r=q2_M(M)
%close all
global ddBmax dt vB vRc0 vRcmax
L=1e4;
pR1n=[L,5*M/7];%红中心点1初始位置坐标[x,y]
pR2n=[L,2*M/7];
pRc1n=[L+200,pR1n(2)];%红运输舰初始位置 在红集群之后200m 
pRc2n=[L+200,pR2n(2)];%以后Rc代表红运输舰，R代表红集群中心
pBn=[0,M*1/2];
rB=100;%蓝最小转弯半径
rR=80;
rRc=500;
vB=25;%蓝速度m/s
vR=20;%红中心点速度m/s
vRc0=13;%红运输舰初始速度
vRcmax=16;%红运输舰最大速度
ddBmax=dt*vB/rB;%蓝方向变化最大角度 
ddRmax=dt*vR/rR;
dt=0.1;%模拟间隔时间/s
dBn=0;%蓝初始方向角
dR1n=atan((pBn(2)-pR1n(2))/(pBn(1)-pR1n(1)))+pi;%红1初始化方向，指向蓝方
dR2n=atan((pBn(2)-pR2n(2))/(pBn(1)-pR2n(1)))+pi;%第23象限角，朝x负方向运动
dRc1n=pi; dRc2n=pi;
Dth=123;
t=0;
% hold on
% axis([0 L+200 0 M])%坐标轴范围
vRc1=vRc0; vRc2=vRc0;
while pBn(1)<L & t<700 & min(norm(pBn-pR1n),norm(pBn-pR2n))>Dth
    pB=pBn;%更新位置
    pR1=pR1n; pR2=pR2n; pRc1=pRc1n; pRc2=pRc2n;
    dB=dBn;%更新方向
    dR1=dR1n; dR2=dR2n; dRc1=dRc1n; dRc2=dRc2n;
    
    dBn=cal_direcB(dB,pB,pR1,pR2,M);%下一蓝速度方向角
    pBn=cal_position(pB,vB,dB);%下一蓝位置坐标
    
    %由蓝的位置坐标计算红的速度方向
    dR1n=cal_direc(dR1,pB,pR1,ddRmax);%下一红1速度方向角
    %输入上一个红的方向角，蓝的位置坐标为目标，红的位置坐标，输出下一个红的方向角
    dR2n=cal_direc(dR2,pB,pR2,ddRmax);
    
    ddRcmax1=dt*vRc1/rRc;
    ddRcmax2=dt*vRc2/rRc;
    dRc1n=cal_direcRc(dRc1,pB,pR1,pRc1,ddRcmax1);%下一红运输舰1速度方向角
    %红集群的位置坐标为目标
    dRc2n=cal_direcRc(dRc2,pB,pR2,pRc2,ddRcmax2);
    
    pR1n=cal_position(pR1,vR,dR1);%由速度方向确定下一红1位置坐标
    pR2n=cal_position(pR2,vR,dR2);
    vRc1=get_vRc(pR1,pB,pRc1);%获得运输舰速度大小 由相对位置决定
    vRc2=get_vRc(pR2,pB,pRc2);
    
    pRc1n=cal_position(pRc1,vRc1,dRc1);%下一红运载舰1位置坐标
    pRc2n=cal_position(pRc2,vRc2,dRc2);
    if min(norm(pRc1n-pB),norm(pRc2n-pB))<1000 
        %disp('蓝与红运载舰距离过近')
        r=2;
        return
    end
    if max(norm(pRc1n-pR1n),norm(pRc2n-pR2n))>2000+Dth
        %disp('红运载舰与集群距离过远')
        r=3;
        return
    end
%     if mod(t,5)<0.1 %5s更新一次图
%         scatter(pBn(1),pBn(2),'b.')
%         scatter(pR1n(1),pR1n(2),'r')
%         scatter(pR2n(1),pR2n(2),'r')
%         scatter(pRc1n(1),pRc1n(2),'g')
%         scatter(pRc2n(1),pRc2n(2),'g')
%     end
%     pause(0.0002)
    t=t+dt;
end
if min(norm(pBn-pR1n),norm(pBn-pR2n))<=Dth
    %disp('successful interception')
    r=0;
elseif t>=700
    disp('time out')
    r=0;
else
    %disp('successful penetration')
    r=1;
end

function dBn=cal_direcB(dB,pB,pRc1,pRc2,M)
%输入上一个蓝的方向角，蓝的位置坐标，红的位置坐标，输出下一个蓝的方向角
global ddBmax vB
Dbr=min(norm(pRc1-pB),norm(pRc2-pB));
D0=400;%临界距离 该距离下蓝方竖直速度最大
Deth=500;%蓝方与边界距离阈值，小于后竖直速度减小
a0=10/M;%0.0001-0.001
if pB(2)<M/2 %下半平面蓝向下运动
    k=-1;
else
    k=1;
end
if Dbr>D0
    vy=k*vB*exp(-a0*(Dbr-D0));
else
    vy=k*vB;
end
if k==1%向上运动
    De=M-pB(2);%到上边界的距离
else%向下
    De=pB(2);
end
if De<Deth
    vy=vy*(-3/Deth^2*De*(De-2*Deth)-2);
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
if Db<Dbth
    k=-1/(Dbth-1000)^2*(Db-1000)*(Db+1000-2*Dbth);
else
    k=1;
end
v=v*k;

function pn=cal_position(p,v,d)
%由当前位置、速度大小、速度方向角计算下一个位置坐标pn
%微元法 每小段近似为直线运动
global dt
vx=v*cos(d);
vy=v*sin(d);
xn=p(1)+vx*dt;
yn=p(2)+vy*dt;
pn=[xn yn];

