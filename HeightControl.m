function h=HeightControl(z,z_d,dz_d)
global l_h
h=-l_h*(z-z_d)+dz_d;