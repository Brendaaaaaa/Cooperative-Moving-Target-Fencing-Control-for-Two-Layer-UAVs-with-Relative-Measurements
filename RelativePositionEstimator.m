function [z_hat]=RelativePositionEstimator11(dt,v,phi,z_hat)
global l3
rhoo=1i*exp(1i*(phi));
% dz_hat=v-gamma_z*rhoo*real(z_hat*conj(rhoo));
dz_hat=v-l3*rhoo*dot(rhoo,z_hat);
z_hat=z_hat+dz_hat*dt;