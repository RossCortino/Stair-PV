function hk = knee_VC_func(phase)
%KNEE_VC_FUNC
%    HK = KNEE_VC_FUNC(PHASE)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    29-Mar-2022 16:27:04

t2 = phase.*pi.*2.0;
t3 = phase.*pi.*4.0;
t4 = phase.*pi.*6.0;
t5 = phase.*pi.*8.0;
t6 = phase.*pi.*1.0e+1;
t7 = phase.*pi.*1.2e+1;
hk = [1.0./2.0,cos(t2),-sin(t2),cos(t3),-sin(t3),cos(t4),-sin(t4),cos(t5),-sin(t5),cos(t6),-sin(t6),cos(t7),-sin(t7),cos(phase.*pi.*1.4e+1)./2.0];
