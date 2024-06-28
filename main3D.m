% 2023-11-09
% Cooperative Moving Target Fencing Control for Two-Layer UAVs with Relative Measurements
clc;
clear all;
%% 参数
% time
t(1)=0;
i=1;
dt=0.001;
t_end=30;

% model
global v_max v_min
v_max=25;
v_min=5;

% RelativePositionEstimator
global l1 l2 l3 l_h
l1=1;l2=1;l3=200;l_h=0.1;

% MaxObs
global omega_max omega_max1
omega_max=(pi/2);
omega_max1=(pi/6);

% MaintainRelativeDistance
T_a=1;
T_zeta=1;
l_ld1=2;l_ld2=5;
l_ff1=1;l_ff2=1;
T_ld=0.1;

% FollowLayer
global l_vk l_omegak
l_vk=1;
l_omegak=1;

% 队形参数

% 初始状态
x_t(i)=0;y_t=0;v_tx(i)=10;v_ty(i)=0;psi_t(i)=0.25*pi;v_t(i)=(v_tx(i)^2+v_ty(i)^2)^0.5;
x_ld(i)=4;y_ld(i)=-12;v_ld(i)=5;psi_ld(i)=-0.3*pi;
x_ff(i)=12;y_ff(i)=-2.4;v_ff(i)=5;psi_ff(i)=0.3*pi;
x_3(i)=0;y_3(i)=8;v_3(i)=5;psi_3(i)=0.5*pi;
x_4(i)=8;y_4(i)=8;v_4(i)=5;psi_4(i)=-0.5*pi;
x_5(i)=-8;y_5(i)=-12;v_5(i)=5;psi_5(i)=0.5*pi;
x_6(i)=-3;y_6(i)=-5;v_6(i)=5;psi_6(i)=0.5*pi;
x_7(i)=2;y_7(i)=-12;v_7(i)=5;psi_7(i)=0.5*pi;
x_8(i)=-8;y_8(i)=-12;v_8(i)=5;psi_8(i)=0.5*pi;
z_t(i)=15;z_ld(i)=16;z_ff(i)=17;z_3(i)=12;z_4(i)=15;z_5(i)=12;z_6(i)=12;z_7(i)=13;z_8(i)=17;


hat_r10(i)=4;hat_gamma_ld(i)=0;hat_rr10(i)=10+2i;y_tt(i)=60/8501*t(i);
%% main
while t(i)<=t_end
%%%%%%%%%%%%%%%%%%% target %%%%%%%%%%%%%%%%%%%
%     omega_t(i)=0;
%     psi_t(i)=sin(0.25*pi*t(i));
%     v_t(i)=13;
    
    if t(i)<=t_end
        ddcx(i)=0.1;ddcy(i)=-0.1;
    else
        ddcx(i)=-0.5;ddcy(i)=0.1;
    end
%     psi_t(i)=
%%%%%%%%%%%%%%%%%%% formation param %%%%%%%%%%%%%%%%%%%
    r_10d(i)=6;
    if t(i)<=1/3*t_end
        kappa(i)=2;
        z_ldd(i)=18;z_ffd(i)=18;z_3d(i)=12;z_4d(i)=13;z_5d(i)=12;z_6d(i)=12;z_7d(i)=12;z_8d(i)=12;
    else
        kappa(i)=4;
        z_ldd(i)=16;z_ffd(i)=16;z_3d(i)=14;z_4d(i)=14.5;z_5d(i)=14;z_6d(i)=14;z_7d(i)=14;z_8d(i)=14;
%     else
%         kappa(i)=6;
    end
    r_21s=6;r_31s=8;r_32s=10;r_42s=7;r_43s=6;r_53s=5;r_54s=5;
%     r_21s=8;r_31s=8;r_32s=8*2^0.5;r_42s=8;r_43s=8;r_53s=8;r_54s=8*2^0.5;
    r_64s=8;r_65s=8;r_71s=8;r_72s=8*2^0.5;r_82s=8;r_83s=8;
%     h_21s=3;h_31s=4;h_32s=5;h_42s=6;h_43s=4;h_53s=5;h_54s=6;h_52s=7;
    r_21d(i)=kappa(i)*r_21s;
    r_31d(i)=kappa(i)*r_31s;
    r_32d(i)=kappa(i)*r_32s;
    r_42d(i)=kappa(i)*r_42s;
    r_43d(i)=kappa(i)*r_43s;
    r_53d(i)=kappa(i)*r_53s;
    r_54d(i)=kappa(i)*r_54s;
    r_64d(i)=kappa(i)*r_64s;
    r_65d(i)=kappa(i)*r_65s;
    r_71d(i)=kappa(i)*r_71s;
    r_72d(i)=kappa(i)*r_72s;
    r_82d(i)=kappa(i)*r_82s;
    r_83d(i)=kappa(i)*r_83s;
    
    
%%%%%%%%%%%%%%%%%%% estimator %%%%%%%%%%%%%%%%%%%
    gamma_ld(i)=atan2(y_t(i)-y_ld(i),x_t(i)-x_ld(i));
    gamma_lf(i)=atan2(y_ld(i)-y_ff(i),x_ld(i)-x_ff(i));
    gamma_ff(i)=atan2(y_t(i)-y_ff(i),x_t(i)-x_ff(i));
    r_10(i)=((-y_t(i)+y_ld(i))^2+(-x_t(i)+x_ld(i))^2)^0.5;
    v_10(i)=-v_t(i)+v_ld(i);
    [hat_r21(i)]=LawofSines(gamma_ld(i),gamma_ff(i),gamma_lf(i),hat_r10(i));

%%%%%%%%%%%%%%%%%%% leader controller %%%%%%%%%%%%%%%%%%%
    xi_ld(i)=abs(psi_ld(i)-gamma_ld(i));
%     xi_ld(i)=psi_ld(i)-gamma_ld(i);
    xi_ld(i)=AngleSat(xi_ld(i));

    tth(i)=hat_r10(i)*sin(xi_ld(i))/r_10d(i);
    th_ld(i)=pi/2-real(acos(tth(i)))-xi_ld(i);
    th_ld(i)=AngleSat(th_ld(i));
    if r_10d(i)<=abs(hat_r10(i)*sin(xi_ld(i)))
        chi_ld(i)=hat_r10(i)*cos(xi_ld(i));
    else
        chi_ld(i)=(r_10d(i)*sin(th_ld(i)))/(sin(xi_ld(i)));
    end
    tau_ld1(i)=-chi_ld(i);
    if i==1
        dhat_r10(i)=0;
    else
        dhat_r10(i)=(hat_r10(i)-hat_r10(i-1))/dt;
    end
    [zeta_ld(i)]=MaintainRelativeDistanceVirtual(T_zeta,r_10d(i),hat_r10(i),dhat_r10(i),xi_ld(i),th_ld(i),tau_ld1(i),l_ld1);
    sat_zeta_ld(i)=InputSat(T_zeta,zeta_ld(i));
    if i==1
        dzeta_ld(i)=0;
    else
        dzeta_ld(i)=(zeta_ld(i)-zeta_ld(i-1))/dt;
    end
    tau_ld2(i)=v_ld(i)-sat_zeta_ld(i);
    [a_ld(i)]=MaintainRelativeDistance(T_a,tau_ld1(i),tau_ld2(i),dzeta_ld(i),l_ld2);
    [omega_ld(i)]=0.5*MaxObs(dt,hat_r10(i),hat_gamma_ld(i),v_t(i),psi_t(i),v_ld(i),psi_ld(i));
    h_ld(i)=HeightControl(z_ld(i),z_ldd(i),0);
    
%%%%%%%%%%%%%%%%%%% first follower controller %%%%%%%%%%%%%%%%%%%
    xi_lf(i)=abs(psi_ff(i)-gamma_lf(i));
    if r_21d(i)<=hat_r21(i)*sin(xi_lf(i))
        chi_ff(i)=hat_r21(i)*cos(xi_lf(i));
        tth_lf(i)=100;
        th_lf(i)=100;
    else
        tth_lf(i)=hat_r21(i)*sin(xi_lf(i))/r_21d(i);
        aa_lf(i)=real(acos(tth_lf(i)));
        th_lf(i)=pi/2-aa_lf(i)-xi_lf(i);
        th_lf(i)=AngleSat(th_lf(i));
        chi_ff(i)=(r_21d(i)*sin(th_lf(i)))/(sin(xi_lf(i)));
    end
    tau_ff1(i)=-chi_ff(i);
    if i==1
        dhat_r21(i)=0;
    else
        dhat_r21(i)=(hat_r21(i)-hat_r21(i-1))/dt;
    end
    [zeta_ff(i)]=MaintainRelativeDistanceVirtual(T_zeta,r_21d(i),hat_r21(i),dhat_r21(i),xi_lf(i),th_lf(i),tau_ff1(i),l_ff1);
    sat_zeta_ff(i)=InputSat(T_zeta,zeta_ff(i));
    if i==1
        dzeta_ff(i)=0;
    else
        dzeta_ff(i)=(zeta_ff(i)-zeta_ff(i-1))/dt;
    end
    tau_ff2(i)=v_ff(i)-sat_zeta_ff(i);
    [a_ff(i)]=MaintainRelativeDistance(T_a,tau_ff1(i),tau_ff2(i),dzeta_ff(i),l_ff2);
    [omega_ff(i)]=0.5*MaxObs(dt,hat_r21(i),gamma_ff(i),v_t(i),psi_t(i),v_ff(i),psi_ff(i));
    h_ff(i)=HeightControl(z_ff(i),z_ffd(i),0);


%%%%%%%%%%%%%%%%%%% follower 1 %%%%%%%%%%%%%%%%%%%
    if i==1
        psi_3d_1(i)=0;
    else
        psi_3d_1(i)=psi_3d(i-1);
    end
    [v_3(i),omega_3(i),psi_3d(i)]=FollowLayer(dt,x_3(i),y_3(i),x_ld(i),y_ld(i),x_ff(i),y_ff(i),r_31d(i),r_32d(i),psi_3(i),psi_3d_1(i));
    h_3(i)=HeightControl(z_3(i),z_3d(i),0);

%%%%%%%%%%%%%%%%%%% follower 2 %%%%%%%%%%%%%%%%%%%
    if i==1
        psi_4d_1(i)=0;
    else
        psi_4d_1(i)=psi_4d(i-1);
    end
    [v_4(i),omega_4(i),psi_4d(i)]=FollowLayer(dt,x_4(i),y_4(i),x_ff(i),y_ff(i),x_3(i),y_3(i),r_42d(i),r_43d(i),psi_4(i),psi_4d_1(i));
    h_4(i)=HeightControl(z_4(i),z_4d(i),0);
    
%%%%%%%%%%%%%%%%%%% follower 3 %%%%%%%%%%%%%%%%%%%
    if i==1
        psi_5d_1(i)=0;
    else
        psi_5d_1(i)=psi_5d(i-1);
    end
    [v_5(i),omega_5(i),psi_5d(i)]=FollowLayer(dt,x_5(i),y_5(i),x_3(i),y_3(i),x_4(i),y_4(i),r_53d(i),r_54d(i),psi_5(i),psi_5d_1(i));
    h_5(i)=HeightControl(z_5(i),z_5d(i),0);

%%%%%%%%%%%%%%%%%%% follower 4 %%%%%%%%%%%%%%%%%%%
    if i==1
        psi_6d_1(i)=0;
    else
        psi_6d_1(i)=psi_6d(i-1);
    end
    [v_6(i),omega_6(i),psi_6d(i)]=FollowLayer(dt,x_6(i),y_6(i),x_4(i),y_4(i),x_5(i),y_5(i),r_64d(i),r_65d(i),psi_6(i),psi_6d_1(i));
    h_6(i)=HeightControl(z_6(i),z_6d(i),0);
    
%%%%%%%%%%%%%%%%%%% follower 5 %%%%%%%%%%%%%%%%%%%
    if i==1
        psi_7d_1(i)=0;
    else
        psi_7d_1(i)=psi_7d(i-1);
    end
    [v_7(i),omega_7(i),psi_7d(i)]=FollowLayer(dt,x_7(i),y_7(i),x_ld(i),y_ld(i),x_ff(i),y_ff(i),r_71d(i),r_72d(i),psi_7(i),psi_7d_1(i));
    h_7(i)=HeightControl(z_7(i),z_7d(i),0);
    
%%%%%%%%%%%%%%%%%%% follower 6 %%%%%%%%%%%%%%%%%%%
    if i==1
        psi_8d_1(i)=0;
    else
        psi_8d_1(i)=psi_8d(i-1);
    end
    [v_8(i),omega_8(i),psi_8d(i)]=FollowLayer(dt,x_8(i),y_8(i),x_ff(i),y_ff(i),x_3(i),y_3(i),r_82d(i),r_83d(i),psi_8(i),psi_8d_1(i));  
    h_8(i)=HeightControl(z_8(i),z_8d(i),0);
    
    

    i=i+1;
    [hat_rr10(i)]=RelativePositionEstimator(dt,v_10(i-1),gamma_ld(i-1),hat_rr10(i-1));
    hat_r10(i)=norm(hat_rr10(i))/2;
    hat_gamma_ld(i)=angle(hat_rr10(i));
    [x_ld(i),y_ld(i),v_ld(i),psi_ld(i)]=UAVLeader(dt,x_ld(i-1),y_ld(i-1),v_ld(i-1),psi_ld(i-1),a_ld(i-1),omega_ld(i-1));
    [x_ff(i),y_ff(i),v_ff(i),psi_ff(i)]=UAVLeader(dt,x_ff(i-1),y_ff(i-1),v_ff(i-1),psi_ff(i-1),a_ff(i-1),omega_ff(i-1));
    [x_3(i),y_3(i),v_3(i),psi_3(i)]=UAVFollower(dt,x_3(i-1),y_3(i-1),v_3(i-1),psi_3(i-1),omega_3(i-1));
    [x_4(i),y_4(i),v_4(i),psi_4(i)]=UAVFollower(dt,x_4(i-1),y_4(i-1),v_4(i-1),psi_4(i-1),omega_4(i-1));
    [x_5(i),y_5(i),v_5(i),psi_5(i)]=UAVFollower(dt,x_5(i-1),y_5(i-1),v_5(i-1),psi_5(i-1),omega_5(i-1));
    [x_6(i),y_6(i),v_6(i),psi_6(i)]=UAVFollower(dt,x_6(i-1),y_6(i-1),v_6(i-1),psi_6(i-1),omega_6(i-1));
    [x_7(i),y_7(i),v_7(i),psi_7(i)]=UAVFollower(dt,x_7(i-1),y_7(i-1),v_7(i-1),psi_7(i-1),omega_7(i-1));
    [x_8(i),y_8(i),v_8(i),psi_8(i)]=UAVFollower(dt,x_8(i-1),y_8(i-1),v_8(i-1),psi_8(i-1),omega_8(i-1));
    
    z_ld(i)=HeightSys(dt,z_ld(i-1),h_ld(i-1));
    z_ff(i)=HeightSys(dt,z_ff(i-1),h_ff(i-1));
    z_3(i)=HeightSys(dt,z_3(i-1),h_3(i-1));
    z_4(i)=HeightSys(dt,z_4(i-1),h_4(i-1));
    z_5(i)=HeightSys(dt,z_5(i-1),h_5(i-1));
    z_6(i)=HeightSys(dt,z_6(i-1),h_6(i-1));
    z_7(i)=HeightSys(dt,z_7(i-1),h_7(i-1));
    z_8(i)=HeightSys(dt,z_8(i-1),h_8(i-1));
    
    [x_t(i),v_tx(i)]=Agent(dt,x_t(i-1),v_tx(i-1),ddcx(i-1));
    [y_t(i),v_ty(i)]=Agent(dt,y_t(i-1),v_ty(i-1),ddcy(i-1));
    z_t(i)=15;
    
    v_t(i)=(v_tx(i)^2+v_ty(i)^2)^0.5;
    psi_t(i)=atan2(y_t(i)-y_t(i-1),x_t(i)-x_t(i-1));
    t(i)=t(i-1)+dt;
    y_tt(i)=-40/8.5*t(i);
    
end
%%
function aa=AngleSat(aa)
if aa>pi
    aa=aa-2*pi;
elseif aa<-pi
    aa=aa+2*pi;
else
    aa=aa;
end
end