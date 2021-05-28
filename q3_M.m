function r=q3_M(M)
global ddBmax dt vB vRc0 vRcmax
L=1e4;
%M=10000;

pR1_1n=[L,7*M/10];%红中心点1初始位置坐标[x,y]
pR2_1n=[L,3*M/10];
pRc1n=[L+200,pR1_1n(2)];%红运输舰初始位置 在红集群之后200m 
pRc2n=[L+200,pR2_1n(2)];%以后Rc代表红运输舰，R代表红集群中心

pR1_2n = pRc1n;    %第二波释放前第二波位置与运输舰一致
pR2_2n = pRc2n;

pBn=[0,M*1/2];
Dwave2 = 6500;%释放第二波x方向阈值距离
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

dBn=0;%蓝初始方向角
% dR1n=atan((pBn(2)-pR1n(2))/(pBn(1)-pR1n(1)))+pi;%红1初始化方向，指向蓝方
% dR2n=atan((pBn(2)-pR2n(2))/(pBn(1)-pR2n(1)))+pi;%第23象限角，朝x负方向运动
dR1_1n=pi; dR2_1n=pi;dR1_2n=pi; dR2_2n=pi;%红方USV初始向左

dRc1n=cal_dRc_init(pRc1n,pBn); 
dRc2n=cal_dRc_init(pRc2n,pBn);%红方运输舰初始方向

Dth1 = 88;Dth2 = 159;%2波次拦截半径

t=0;
vRc1=vRc0; vRc2=vRc0;
wave2 = [0 0];%初始化判断是否已经生成第二波
while pBn(1)<L & t<700 & min(norm(pBn-pR1_1n),norm(pBn-pR2_1n))>Dth1 &...
        min(norm(pBn-pR1_2n),norm(pBn-pR2_2n))>Dth2
    %更新位置
    pB=pBn;
    pR1_1=pR1_1n; pR2_1=pR2_1n; pRc1=pRc1n; pRc2=pRc2n;
    pR1_2=pR1_2n; pR2_2=pR2_2n;%第二波
    
    %更新方向
    dB=dBn;
    dR1_1=dR1_1n; dR2_1=dR2_1n; dRc1=dRc1n; dRc2=dRc2n;
    dR1_2=dR1_2n; dR2_2=dR2_2n;
    
    %判断释放第二波
    while wave2(1)==0 & pB(1)>pR1_1(1)-Dwave2  
        %蓝方USV快突破第1艘运输舰的第一波次,生成第二波次
        if pB(2)>=pRc1+200   %Y方向蓝方和运输舰距离过大且在上方
            pR1_2 = [pRc1(1) pRc1(2)+200];%尽量向上放出
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
        %蓝方USV突破第2艘运输舰的第一波次,生成第二波次
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
    
    dBn=cal_direcB(dB,pB,pR1_1,pR2_1,pR1_2,pR2_2,M);%下一蓝速度方向角
    pBn=cal_position(pB,vB,dB);%下一蓝位置坐标
    
    %由蓝的位置坐标计算红的速度方向 
    %输入上一个红的方向角，蓝的位置坐标为目标，红的位置坐标，输出下一个红的方向角
%     dR1_1n=cal_direcR_1(dR1_1,pB,pR1_1,ddRmax,50,wave2(1)); %下一红1_1速度方向角
%     dR2_1n=cal_direcR_1(dR2_1,pB,pR2_1,ddRmax,50,wave2(2)); %下一红2_1速度方向角
    dR1_1n=cal_direc(dR1_1,pB,pR1_1,ddRmax); %下一红1_1速度方向角
    dR2_1n=cal_direc(dR2_1,pB,pR2_1,ddRmax); %下一红2_1速度方向角
    if wave2(1)==1
        dR1_2n=cal_direcR(dR1_2,pB,pR1_2,ddRmax,50);%下一红1_2速度方向角
        pR1_2n=cal_position(pR1_2,vR,dR1_2);%由速度方向确定下一红1_2位置坐标
    end
    if wave2(2)==1
        dR2_2n=cal_direcR(dR2_2,pB,pR2_2,ddRmax,50);%下一红2_2速度方向角
        pR2_2n=cal_position(pR2_2,vR,dR2_2);%由速度方向确定下一红2_2位置坐标
    end
    
    ddRcmax1=dt*vRc1/rRc;
    ddRcmax2=dt*vRc2/rRc;
    dRc1n=cal_direc(dRc1,(pR1_1+pR1_2)/2,pRc1,ddRcmax1);
    dRc2n=cal_direc(dRc2,(pR2_1+pR2_2)/2,pRc2,ddRcmax2);
    
    pR1_1n=cal_position(pR1_1,vR,dR1_1);%由速度方向确定下一红1位置坐标
    pR2_1n=cal_position(pR2_1,vR,dR2_1);
    
    
    vRc1=get_vRc(pR1_1,pB,pRc1);%获得运输舰速度大小 由相对位置决定
    vRc2=get_vRc(pR2_1,pB,pRc2);         %第二波近，暂不考虑
    
    pRc1n=cal_position(pRc1,vRc1,dRc1);%下一红运载舰1位置坐标
    pRc2n=cal_position(pRc2,vRc2,dRc2);
    if min(norm(pRc1n-pB),norm(pRc2n-pB))<1000 
        %disp('蓝与红运载舰距离过近')
        r=2;
        return
    end
    if max(norm(pRc1n-pR1_1n),norm(pRc2n-pR2_1n))>2000+Dth1
        %disp('红运载舰与集群距离过远')
        r=3;
        return
    end
%     if mod(t,5)<0.1 %5s更新一次图
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
%输入上一个蓝的方向角，蓝的位置坐标，红的位置坐标，输出下一个蓝的方向角
global ddBmax vB
Dbr=min([norm(pR1-pB),norm(pR2-pB),norm(pR3-pB),norm(pR4-pB)]);
D0=600;%临界距离 该距离下蓝方竖直速度最大
Deth=500;%蓝方与边界距离阈值，小于后竖直速度减小
a0=10/M;
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

function pn=cal_position(p,v,d)
%由当前位置、速度大小、速度方向角计算下一个位置坐标pn
%微元法 每小段近似为直线运动
global dt
vx=v*cos(d);
vy=v*sin(d);
xn=p(1)+vx*dt;
yn=p(2)+vy*dt;
pn=[xn yn];

