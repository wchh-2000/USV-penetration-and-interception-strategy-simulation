function dir_n=cal_direc(dir,p_dst,p_self,ddirmax)
%�˶�����ֱ��ָ��Ŀ��
%���뵱ǰ����ǣ�Ŀ�����꣬�Լ����꣬���ת��Ƕȣ������һ�����
% dis=dis_p2line(pR,tan(dR),pB);
% dd=asin(dis/norm(pR-pB));%����Ǳ仯��
if p_dst(1)-p_self(1)<0%Ŀ������С���Լ���x���꣬��x�������˶�����23���޽�
    dir_n=atan((p_dst(2)-p_self(2))/(p_dst(1)-p_self(1)))+pi;
elseif p_dst(1)-p_self(1)>0%��14���޽ǣ���x�������˶�
    dir_n=atan((p_dst(2)-p_self(2))/(p_dst(1)-p_self(1)));
    if dir_n<0
        dir_n=dir_n+2*pi;%arctan�������תΪ����
    end
elseif p_dst(1)-p_self(1)==0
    if p_dst(2)-p_self(2)<0 %y�����С
        dir_n=3*pi/2;
    else
        dir_n=pi/2;
    end
end
if abs(dir_n-dir)>ddirmax %���ת��Ƕ�����
    if dir_n>dir
        dir_n=dir+ddirmax;
    else
        dir_n=dir-ddirmax;
    end
end