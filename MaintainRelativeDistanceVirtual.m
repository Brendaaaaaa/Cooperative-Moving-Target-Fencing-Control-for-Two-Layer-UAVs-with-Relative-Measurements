function [zeta]=MaintainRelativeDistanceVirtual(T,r_d,hat_r,dhat_r,xi,th,tau_1,l_1)
% xi=abs(psi-gamma);
% th=pi/2-acos(hat_r*sin(xi)/r_d)-xi;

if r_d<=abs(hat_r*sin(xi))
    mm=dhat_r*cos(xi);
else
    mm=(r_d*cos(th)*dhat_r)/(r_d^2-(hat_r*sin(xi))^2)^0.5;
end

zeta=1/T*(-l_1*tau_1+mm);