function [x,y,v,psi]=UAVFollower(dt,x,y,v,psi,omega)
global v_min v_max omega_max1

if omega>=omega_max1
    omega=omega_max1;
elseif omega<=-omega_max1
    omega=-omega_max1;
else
    omega=omega;
end

dx=v*cos(psi);
dy=v*sin(psi);
dpsi=omega;

x=x+dt*dx;
y=y+dt*dy;
psi=psi+dt*dpsi;

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
