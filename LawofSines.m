function [hat_r21]=LawofSines(gamma_ld,gamma_ff,gamma_lf,hat_r10)
gamma_0=gamma_ff-gamma_ld;
gamma_1=gamma_ff-gamma_lf;

hat_r21=hat_r10*sin(gamma_0)/sin(gamma_1);