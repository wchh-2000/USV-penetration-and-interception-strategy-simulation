function dRc2n_init = cal_dRc_init(pRcn,pBn)
if pRcn(2) == pBn(2)
    dRc2n_init = pi;
else
    if pRcn(2) > pBn(2)%���佢��
    dRc2n_init = 4*pi/3;
    else %��USV��
    dRc2n_init = 2*pi/3;
    end
end