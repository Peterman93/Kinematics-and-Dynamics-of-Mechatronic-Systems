% formulation of the Dynamic Equations of Motion
% for the RRP kinematic structure
clear all
syms Ek1 Ek2 Ek3 Ek m1 m2 m3 v2 v3 J2 s2 q1 q2 q3 dq1 dq2 dq3 real
syms Ep1 Ep2 Ep3 g real
syms L t ddq1 ddq2 ddq3 S1 S2 S3 S4 real
% kinetic energy
v2 = sqrt((s2*dq2+dq1*sin(q2))^2+(dq1*cos(q2))^2)
v3 = sqrt((dq2*q3+dq3+dq1*sin(q2))^2+(dq1*cos(q2)+dq3)^2)
Ek1=(1/2)*m1*(dq1)^2
Ek2=(1/2)*m2*(v2)^2+(1/2)*J2*(dq2)^2
Ek3=(1/2)*m3*(v3)^2
Ek=simplify(Ek1+Ek2+Ek3)
% Potential energy
Ep1=m1*g*q1
Ep2=m2*g*q1+m2*g*s2*sin(q2)
Ep3=m3*g*q1+m3*g*q3*sin(q2)
Ep=simplify(Ep1+Ep2+Ep3)
% Lagrange function
L=Ek-Ep
% Lagrange Equation
% differentiate L with respect to (wrt) dqi i=1,2,3
f11=diff(L,'dq1')
f21=diff(L,'dq2')
f31=diff(L,'dq3')
% introduce time vatiable t
S1={'q1','q2','q3','dq1','dq2','dq3'};
S2={'q1(t)','q2(t)','q3(t)','dq1(t)','dq2(t)','dq3(t)'};
f12=subs(f11,S1,S2)
f22=subs(f21,S1,S2)
f32=subs(f31,S1,S2)
% differentiate fi2 wrt t - time i=1,2,3
f13=diff(f12,'t')
f23=diff(f22,'t')
f33=diff(f32,'t')
% introduce common denotations and remove t variable
S3={'diff(q1(t),t)','diff(q2(t),t)','diff(q3(t),t)','diff(dq1(t),t)','diff(dq2(t),t)','diff(dq3(t),t)'};
S4={'dq1','dq2','dq3','ddq1','ddq2','ddq3'};
f14=subs(f13,S3,S4)
f24=subs(f23,S3,S4)
f34=subs(f33,S3,S4)
S5={'dq1(t)','dq2(t)','dq3(t)'};
S6={'dq1','dq2','dq3'};
f15=subs(f14,S5,S6)
f25=subs(f24,S5,S6)
f35=subs(f34,S5,S6)
S7={'q1(t)','q2(t)','q3(t)'};
S8={'q1','q2','q3'};
r11=subs(f15,S7,S8)
r21=subs(f25,S7,S8)
r31=subs(f35,S7,S8)
% differentiate L wrt qi i=1,2,3
r12=diff(L,'q1')
r22=diff(L,'q2')
r32=diff(L,'q3')
% join components of the left hand side of the dynamic equations of motion
r1=simplify(r11-r12)
r2=simplify(r21-r22)
r3=simplify(r31-r32)