function [v_k,omega_k,psi_kd]=FollowLayer(dt,pp_kx,pp_ky,pp_mx,pp_my,pp_nx,pp_ny,r_dm,r_dn,psi_k,psi_kd_1)
global l_vk l_omegak
rr_kmx=pp_kx-pp_mx;
rr_kmy=pp_ky-pp_my;
rr_knx=pp_kx-pp_nx;
rr_kny=pp_ky-pp_ny;
tilde_r_km=rr_kmx^2+rr_kmy^2-r_dm^2;
tilde_r_kn=rr_knx^2+rr_kny^2-r_dn^2;

u_kx=-l_vk*(tilde_r_km*rr_kmx+tilde_r_kn*rr_knx);
u_ky=-l_vk*(tilde_r_km*rr_kmy+tilde_r_kn*rr_kny);

if u_kx^2+u_ky^2==0
    psi_kd=0;
else
    psi_kd=atan2(u_ky,u_kx);
end
tilde_psi_k=psi_k-psi_kd;
dpsi_kd=(psi_kd-psi_kd_1)/dt;

uu=InputSat(1000,(u_kx^2+u_ky^2)^0.5);
v_k=uu*cos(tilde_psi_k);
omega_k=-l_omegak*tilde_psi_k+dpsi_kd;