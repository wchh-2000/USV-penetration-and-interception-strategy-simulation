function S1 %������ط�Χ���
p1=[50 0];
r72=pi*72/180;%����
r36=r72/2;
p2=[50+100*cos(r72),100*sin(r72)];
p3=[0 100*(sin(r36)+sin(r72))];
p4=[-p2(1) p2(2)];
p5=-p1;
S1=cal_S([p1;p2;p3;p4;p5],-160,160,-100,250)
r1=sqrt(S1/pi)
