function d=dis_p2line(p1,p2,k)
%计算点p2到直线（过p1,斜率k）距离d  k=tan(theta)
x1=p1(1); y1=p1(2);
x2=p2(1); y2=p2(2);
a=-k; c=k*x1-y1;
d=abs((a*x2+y2+c)/sqrt(a^2+1));