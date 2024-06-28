function [a]=MaintainRelativeDistance(T,tau_1,tau_2,dzeta,l_2)
a=1/T*(-tau_1-l_2*tau_2+T*dzeta);