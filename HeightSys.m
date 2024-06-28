function z=HeightSys(dt,z,h)
if h>=0.13
    h=0.13;
elseif h<=-0.13
    h=-0.13;
else
    h=h;
end
dz=h;
z=z+dt*dz;