function r=q3_1(M)%区域防守
global ddBmax dt vB L vRc0 vRcmax
L=1e4;
%M=10000;
% close all
% hold on
% axis([0 L+200 0 M])%坐标轴范围
%初始位置，红方一次性释放
pRc1n = [L,7.5*M/10];
pRc2n = [L,2.5*M/10];
pR1n = [L,7.5*M/10 + 200];
pR2n = [L,7.5*M/10 - 200];%12对应运输舰1
pR3n = [L,2.5*M/10 + 200];
pR4n = [L,2.5*M/10 - 200];%34对应运输舰2
pBn = [0,M*1/2];
pD1n = [L/4 4*M/5];    %初始防守阵地坐标
pD2n = [L/4 3*M/5];
pD3n = [L/4 2*M/5];
pD4n = [L/4 1*M/5];
pD10 = [L*2/3 9*M/10];    %初始防守阵地坐标
pD20 = [L/2 3*M/5];
pD30 = [L/2 2*M/5];
pD40 = [L*2/3 1*M/10];

rB=100;%蓝最小转弯半径
rR=80;
rRc=500;
vB=25;%蓝速度m/s
vR=20;%红中心点速度m/s
vRc0=0;%红运输舰初始速度
vRcmax=16;%红运输舰最大速度
ddBmax=dt*vB/rB;%蓝方向变化最大角度 
ddRmax=dt*vR/rR;
dt=1;%模拟间隔时间

%初始方向角
dBn=0;
% dR1n=atan((pBn(2)-pR1n(2))/(pBn(1)-pR1n(1)))+pi;%红1初始化方向，指向蓝方
% dR2n=atan((pBn(2)-pR2n(2))/(pBn(1)-pR2n(1)))+pi;%第23象限角，朝x负方向运动
dR1n=pi; dR2n=pi;dR3n=pi; dR4n=pi;%所有红方USV初始向左

dRc1n=cal_dRc_init(pRc1n,pBn); 
dRc2n=cal_dRc_init(pRc2n,pBn);%红方运输舰初始方向

Dth = 123.5;%拦截半径
D_attack = 1000;  %蓝方距离其中一个防区距离很近时集体发起攻击

t=0;
vRc1=vRc0; vRc2=vRc0;
attack = 0;%初始化状态，0为未发起总攻，1为发起总攻后
while pBn(1)<L & t<700 & ...
    min([norm(pBn-pR1n),norm(pBn-pR2n),norm(pBn-pR3n),norm(pBn-pR4n)])>Dth 
       
    %更新位置
    pB=pBn;
    pR1=pR1n; pR2=pR2n;  pR3=pR3n; pR4=pR4n;
    pRc1=pRc1n; pRc2=pRc2n;
%     pD1=pD1n;pD2=pD2n;pD3=pD3n;pD4=pD4n;
    
%     if min([norm(pBn-pD1n),norm(pBn-pD2n),norm(pBn-pD3n),norm(pBn-pD4n)])<D_attack
    if min([norm(pBn-pD10),norm(pBn-pD20),norm(pBn-pD30),norm(pBn-pD40)])<D_attack
        attack = 1;
    end
    %更新方向
    dB=dBn;
    dR1=dR1n; dR2=dR2n; dR3=dR3n; dR4=dR4n;
    dRc1=dRc1n; dRc2=dRc2n;
    
    dBn=cal_direcB(dB,pB,pR1,pR2,pR3,pR4,M);%下一蓝速度方向角
    pBn=cal_position(pB,vB,dB);%下一蓝位置坐标
    
    %区域防守算法计算红的速度方向
    %输入上一个红的方向角，目标为防守阵地，输出下一个红的方向角
    if attack == 0      %2种阶段红方USV的目标
        dR1n=cal_direc(dR1,pD10,pR1,ddRmax);   %下一红1速度方向角
        dR2n=cal_direc(dR2,pD20,pR2,ddRmax);   %下一红2速度方向角
        dR3n=cal_direc(dR3,pD30,pR3,ddRmax);   %下一红3速度方向角
        dR4n=cal_direc(dR4,pD40,pR4,ddRmax);   %下一红4速度方向角
    else
        dR1n=cal_direc(dR1,pB,pR1,ddRmax);   %下一红1速度方向角
        dR2n=cal_direc(dR2,pB,pR2,ddRmax);   %下一红2速度方向角
        dR3n=cal_direc(dR3,pB,pR3,ddRmax);   %下一红3速度方向角
        dR4n=cal_direc(dR4,pB,pR4,ddRmax);   %下一红4速度方向角
    end
    
    pR1n=cal_position(pR1,vR,dR1);%由速度方向确定下一红1位置坐标
    pR2n=cal_position(pR2,vR,dR2);%由速度方向确定下一红2位置坐标
    pR3n=cal_position(pR3,vR,dR3);%由速度方向确定下一红3位置坐标
    pR4n=cal_position(pR4,vR,dR4);%由速度方向确定下一红4位置坐标
%     pD1n = cal_pD(pD1,pB);          %确定下一时刻目标防区位置坐标
%     pD2n = cal_pD(pD2,pB);
%     pD3n = cal_pD(pD3,pB);
%     pD4n = cal_pD(pD4,pB);
    
    ddRcmax1=dt*vRc1/rRc;
    ddRcmax2=dt*vRc2/rRc;
    dRc1n=cal_direc(dRc1,(pR1+3*pR2)/4,pRc1,ddRcmax1);%下一红运输舰1速度方向角
    dRc2n=cal_direc(dRc2,(pR4+3*pR3)/4,pRc2,ddRcmax2);%下一红运输舰2速度方向角
    
   
    vRc1=get_vRc(pR1,pB,pRc1);%获得运输舰速度大小 由相对位置决定
    vRc2=get_vRc(pR2,pB,pRc2);         %第二波近，暂不考虑
    
    pRc1n=cal_position(pRc1,vRc1,dRc1);%下一红运载舰1位置坐标
    pRc2n=cal_position(pRc2,vRc2,dRc2);%下一红运载舰2位置坐标
    
    if min(norm(pRc1n-pB),norm(pRc2n-pB))<1000 
        %disp('蓝与红运载舰距离过近')
        r=2;
        return
    end
    if max([norm(pRc1n-pR1n),norm(pRc1n-pR2n),norm(pRc2n-pR3n),norm(pRc2n-pR4n)])>2000+Dth
        %disp('红运载舰与集群距离过远')
        r=3;
        return
    end
%     if mod(t,5)<0.1 %5s更新一次图
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
%输入上一个蓝的方向角，蓝的位置坐标，红的位置坐标，输出下一个蓝的方向角
global ddBmax vB
Dbr=min([norm(pR1-pB),norm(pR2-pB),norm(pR3-pB),norm(pR4-pB)]);
D0=300;%临界距离 该距离下蓝方竖直速度最大
Deth=500;%蓝方与边界距离阈值，小于后竖直速度减小
a0=0.001;
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
%和B距离判定
if Db<Dbth
    k=-1/(Dbth-1000)^2*(Db-1000)*(Db+1000-2*Dbth);
else
    k=1;
end
v=v*k;
end
function pn=cal_position(p,v,d)
%由当前位置、速度大小、速度方向角计算下一个位置坐标pn
%微元法 每小段近似为直线运动
global dt
vx=v*cos(d);
vy=v*sin(d);
xn=p(1)+vx*dt;
yn=p(2)+vy*dt;
pn=[xn yn];
end
function pDn = cal_pD(pD,pB)
global L
%由当前阵地位置和蓝USV位置变化出下一个阵地位置，移动量是距离的函数
d_DB = norm(pD-pB);
bP = L*1/4;         %防区位置平移函数为直线，下面表示截距和斜率
kP = -bP/7500;
Dx = kP*d_DB+bP;
pDn = [Dx+pD(1),pD(2)];
end