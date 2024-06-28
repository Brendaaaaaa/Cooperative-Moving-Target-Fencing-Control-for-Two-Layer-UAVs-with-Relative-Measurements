function [x,y,v,psi]=UAVLeader(dt,x,y,v,psi,a,omega)
global v_max v_min

dx=v*cos(psi);
dy=v*sin(psi);
dv=a;
dpsi=omega;

x=x+dt*dx;
y=y+dt*dy;
v=v+dt*dv;
psi=omega;%psi+dt*dpsi;

if v>=v_max
    v=v_max;
elseif v<=v_min
    v=v_min;
else
    v=v;
end

while psi>pi
    psi=psi-2*pi;
end
while psi<-pi
    psi=psi+2*pi;
end
