function dir_n=cal_direcR(dir,p_dst,p_self,ddirmax,d_rush)
%运动方向一阶段对齐Y方向，二阶段直接指向目标
%输入当前方向角，目的坐标，自己坐标，最大转向角度，冲刺切换的Y距离阈值（一般很小）
%输出下一方向角
if abs(p_dst(2)-p_self(2)) <= d_rush    %y轴接近，
    dir_n = cal_direc(dir,p_dst,p_self,ddirmax);
else
    if p_dst(2) > p_self(2) %对方高，向上对准
        dir_n = pi/2;
    else
        dir_n = 3*pi/2;
    end
    
    if abs(dir_n-dir)>ddirmax %最大转向角度限制
        if dir_n>dir
            dir_n=dir+ddirmax;
        else
            dir_n=dir-ddirmax;
        end
    end
end
    