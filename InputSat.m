function [sat]=InputSat(T,u)
sat=T*tanh(u/T);
