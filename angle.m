function a=angle(d,s)
%a: the direction of self to destination(rad)
if d(1)-s(1)<0
    a=atan((d(2)-s(2))/(d(1)-s(1)))+pi;
elseif d(1)-s(1)>=0%��14���޽ǣ���x�������˶�
    a=atan((d(2)-s(2))/(d(1)-s(1)));
    if a<0
        a=a+2*pi;%arctan�������תΪ����
    end
end