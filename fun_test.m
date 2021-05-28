function fun_test
% x=0:0.02:1;
% y1=sqrt(x);
% y2=-x.^2+2*x;
% plot(x,y1,'r',x,y2,'b')
% figure(1)
% legend('y1=sqrt(x)','y2=-x.^2+2*x')
% figure(2)
% x=0:10:2000;
% y=3./(1+exp(-x/100+5))+13;
% plot(x,y)
% xlabel('Dr') 
% ylabel('vRc')
% figure(3)
% D=1500;
% Db=1000:10:2*D-1000;
% n=length(Db);
% k=zeros(1,n);
% for i=1:n
%     if Db(i)<D
%         k(i)=-1/(D-1000)^2*(Db(i)-1000)*(Db(i)+1000-2*D);
%     else
%         k(i)=1;
%     end
% end
% plot(Db,k)
% xlabel('Db')
% ylabel('kd')
% axis([500 2*D-1000 0 1.5])
% Deth=2000;
% De=0:10:2*Deth;
% n=length(De);
% ke=zeros(1,n);
% for i=1:n
%     if De(i)<Deth
%         ke(i)=-3/Deth^2*De(i)*(De(i)-2*Deth)-2;
%     else
%         ke(i)=1;
%     end
% end
% plot(De,ke)
% xlabel('De/m')
% ylabel('ke')
% axis([0 2*Deth -2 1.5])
D01=200;
D02=600;
vB=25;
a01=0.0005;
a02=0.001;
Dbr=0:10:8000;
n=length(Dbr);
vy1=zeros(1,n);
vy2=zeros(1,n);
for i=1:n
    if Dbr(i)>D01
        vy1(i)=vB*exp(-a02*(Dbr(i)-D01));
    else
        vy1(i)=vB;
    end
end
for i=1:n
    if Dbr(i)>D02
        vy2(i)=vB*exp(-a02*(Dbr(i)-D02));
    else
        vy2(i)=vB;
    end
end
hold on
plot(Dbr,vy1,'r',Dbr,vy2,'b')
%legend('a0=0.0005','a0=0.001')
legend('D0=200','D0=600')
xlabel('Dbr/m')
ylabel('vy')
axis([0 8000 0 30])
