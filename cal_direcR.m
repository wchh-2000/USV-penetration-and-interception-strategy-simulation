function dir_n=cal_direcR(dir,p_dst,p_self,ddirmax,d_rush)
%�˶�����һ�׶ζ���Y���򣬶��׶�ֱ��ָ��Ŀ��
%���뵱ǰ����ǣ�Ŀ�����꣬�Լ����꣬���ת��Ƕȣ�����л���Y������ֵ��һ���С��
%�����һ�����
if abs(p_dst(2)-p_self(2)) <= d_rush    %y��ӽ���
    dir_n = cal_direc(dir,p_dst,p_self,ddirmax);
else
    if p_dst(2) > p_self(2) %�Է��ߣ����϶�׼
        dir_n = pi/2;
    else
        dir_n = 3*pi/2;
    end
    
    if abs(dir_n-dir)>ddirmax %���ת��Ƕ�����
        if dir_n>dir
            dir_n=dir+ddirmax;
        else
            dir_n=dir-ddirmax;
        end
    end
end
    