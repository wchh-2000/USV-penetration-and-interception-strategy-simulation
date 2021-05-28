function find_fastest()
lb=[130,500,-400,0.0001];%130<D0<400,500<Deth<3000,-400<Dx<100,0.0001<a0<0.002
ub=[400,3000,100,0.002];
X0=[390,2000,0,0.001];
options = optimoptions('fmincon','Display','iter');
options.MaxFunEvals = 1e8;
[X,t,f]=fmincon(@q2_t,X0,[],[],[],[],lb,ub,[],options);
D0=X(1) 
Deth=X(2) 
Dx=X(3) 
a0=X(4)
t
f
   