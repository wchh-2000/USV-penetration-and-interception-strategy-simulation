function dn=cal_direcRc(d,pB,pR,pRc,ddmax)
%�����ؽ��˶�����
if norm(pB-pRc)<1500
    dn=angle(pB,pRc)+pi;%����
else
    dn=angle(pR,pRc);%һ������¸���켯Ⱥ
end
if abs(dn-d)>ddmax %���ת��Ƕ�����
    if dn>d
        dn=d+ddmax;
    else
        dn=d-ddmax;
    end
end
    