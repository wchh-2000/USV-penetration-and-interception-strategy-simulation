function dir_n=cal_direc(dir,p_dst,p_self,ddirmax)
%运动方向直接指向目标
%输入当前方向角，目的坐标，自己坐标，最大转向角度，输出下一方向角
% dis=dis_p2line(pR,tan(dR),pB);
% dd=asin(dis/norm(pR-pB));%方向角变化量
if p_dst(1)-p_self(1)<0%目的坐标小于自己的x坐标，朝x负方向运动，第23象限角
    dir_n=atan((p_dst(2)-p_self(2))/(p_dst(1)-p_self(1)))+pi;
elseif p_dst(1)-p_self(1)>0%第14象限角，朝x正方向运动
    dir_n=atan((p_dst(2)-p_self(2))/(p_dst(1)-p_self(1)));
    if dir_n<0
        dir_n=dir_n+2*pi;%arctan输出负的转为正的
    end
elseif p_dst(1)-p_self(1)==0
    if p_dst(2)-p_self(2)<0 %y坐标减小
        dir_n=3*pi/2;
    else
        dir_n=pi/2;
    end
end
if abs(dir_n-dir)>ddirmax %最大转向角度限制
    if dir_n>dir
        dir_n=dir+ddirmax;
    else
        dir_n=dir-ddirmax;
    end
end