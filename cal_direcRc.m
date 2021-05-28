function dn=cal_direcRc(d,pB,pR,pRc,ddmax)
%红运载舰运动方向
if norm(pB-pRc)<1500
    dn=angle(pB,pRc)+pi;%反向
else
    dn=angle(pR,pRc);%一般情况下跟随红集群
end
if abs(dn-d)>ddmax %最大转向角度限制
    if dn>d
        dn=d+ddmax;
    else
        dn=d-ddmax;
    end
end
    