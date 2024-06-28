function [omega]=MaxObs(dt,hat_r10,hat_gamma,vt,psi_t,v,psi_ld)
global omega_max
x=hat_r10*cos(hat_gamma);   % hat_rr的坐标:hat_r10长度；hat_gamma角度
y=hat_r10*sin(hat_gamma);
d_x=dt*vt*cos(psi_t);   % dt内target的位移
d_y=dt*vt*sin(psi_t);

a=hat_r10^2*(-y-2*d_y)+2*x*d_x*d_y-y*(d_y^2-d_x^2+(v*dt)^2);
b=hat_r10^2*(-x-2*d_x)+2*y*d_x*d_y-x*(d_x^2-d_y^2+(v*dt)^2);

ee=asin(b/(a^2+b^2)^0.5);   % 论文里的epsilon
rr_ld=hat_r10*exp(hat_gamma*1i);  % k时刻的相对位置（向量形式）
bar_rr=-rr_ld-dt*vt*exp(psi_t*1i);  % 虚拟位移（向量形式）
aa=2*v*dt*(real(rr_ld*bar_rr))/(a^2+b^2)^0.5;
psi_d1=real(asin(aa))-ee; % 最优解1
psi_d2=pi-real(asin(aa))-ee;  % 最优解2



%%%%%%%%%%%%%%%%%%%%% 后面加的 %%%%%%%%%%%%%%%%%%%%%
% psi_p=psi_ld+omega_max;
% psi_n=psi_ld-omega_max;
% bar_r1=bar_rr+v*dt*exp(psi_d1*1i);theta_ld1=abs(angle(bar_r1)-hat_gamma);   % 最优解1对应的k+1时刻相对位置（向量形式）
% bar_r2=bar_rr+v*dt*exp(psi_d2*1i);theta_ld2=abs(angle(bar_r2)-hat_gamma);   % 最优解2对应的k+1时刻相对位置（向量形式）
% bar_rp=bar_rr+v*dt*exp(psi_p*1i);theta_ldp=abs(angle(bar_rp)-hat_gamma);   % 可行域正的max对应的k+1时刻相对位置（向量形式）
% bar_rn=bar_rr+v*dt*exp(psi_n*1i);theta_ldn=abs(angle(bar_rn)-hat_gamma);   % 可行域正的max对应的k+1时刻相对位置（向量形式）
% J_psi1=(sin(theta_ld1)/norm(bar_r1))^2; % 最优解1对应的代价
% J_psi2=(sin(theta_ld2)/norm(bar_r2))^2; % 最优解2对应的代价
% J_pmax=(sin(theta_ldp)/norm(bar_rp))^2; % 可行域正的max对应代价
% J_nmax=(sin(theta_ldn)/norm(bar_rn))^2; % 可行域负的max对应代价
% if J_pmax>J_nmax    % 先选出可行域边界的最大值
%     J_max=J_pmax;
%     psi_max=psi_p;
% else
%     J_max=J_nmax;
%     psi_max=psi_n;
% end
% 
% if abs(psi_d1-psi_ld)<=abs(psi_d2-psi_ld)   % 比较k时刻哪个最优解离当前航向近，近的那个设为bar_omega
%     bar_omega=psi_d1;
%     J_bar=J_psi1;
% else
%     bar_omega=psi_d2;
%     J_bar=J_psi2;
% end
% % 判断两个最优解在不在可行域内（参考he）
% if abs(psi_d1-psi_ld)<=omega_max && abs(psi_d2-psi_ld)<=omega_max    % 两个都在
%     if J_psi1==J_psi2    % 若二者代价相等，取离当前航向近的，即bar_omega
%         psi_ld_n=bar_omega;
%     elseif J_psi1>J_psi2    % 不相等则取代价大的
%         psi_ld_n=psi_d1;
%     else
%         psi_ld_n=psi_d2;
%     end
% elseif abs(bar_omega-psi_ld)<=omega_max    % 离得近的那个在，比较近的和边界大的
%     if J_bar>=J_max
%         psi_ld_n=bar_omega;
%     else
%         psi_ld_n=psi_max;
%     end
% else
%     psi_ld_n=psi_max;
% end
% omega=(psi_ld_n-psi_ld)/dt;
    
        
    
    

if (psi_d1>=psi_ld-omega_max)&&(psi_d1<=psi_ld+omega_max)
    flag_1=1;
else
    flag_1=0;
    if psi_d1>psi_ld+omega_max
        psi_d1=psi_ld+omega_max;
    else
        psi_d1=psi_ld-omega_max;
    end
end

if (psi_d2>=psi_ld-omega_max)&&(psi_d2<=psi_ld+omega_max)
    flag_2=1;
else
    flag_2=0;
    if psi_d2>psi_ld+omega_max
        psi_d2=psi_ld+omega_max;
    else
        psi_d2=psi_ld-omega_max;
    end
end

if (flag_1==1)&&(flag_2==1)
    if abs(psi_d1-psi_ld)>=abs(psi_d2-psi_ld)
        omega=psi_d1;
    else
        omega=psi_d2;
    end
elseif (flag_1==1)&&(flag_2==0)
    omega=psi_d1;
elseif (flag_2==1)&&(flag_1==0)
    omega=psi_d2;
else
    omega=psi_d1;
end
