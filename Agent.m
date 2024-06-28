function [z,v]=Agent(dt,z,v,u)
dz=v;
dv=u;
z=z+dz*dt;
v=v+dv*dt;