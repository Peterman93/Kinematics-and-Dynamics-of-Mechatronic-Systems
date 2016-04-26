% this script determines a Homogeneous Transformation matrix
clear all
% declaration of symbols
syms th1 d1 th2 a3 th4 d5 q1 q2 q3
% determination of a symbolic form of HT matrices –
% application of mA function
A1=mA(th1,d1,0,sym(pi/2));
A2=mA(th2,0,0,0);
A3=mA(0,0,a3,0);
A4=mA(th4,0,0,sym(pi/2));
A5=mA(0,d5,0,0);
% multiplication of matrices
T05=A1*A2*A3*A4*A5
% substitution of rotational joint variables
% for the simplification purpose
T05v=subs(T05,{th1,th2,th4},{q1,q2,q3})
% indication of joint coordinates
% variables: th1,th2 and a3 indicated by ‘1’s
zmie=[[1,0,0,0];[1,0,0,0];[0,0,1,0];[1,0,0,0];[0,0,0,0]];
% a simplified form of the evaluated HT matrices
% for interpretation purpose for a user
T05u=zam(zmie,T05v,'q')

% example of substitution of the join variables values
% and constant values into the T0e matrix for the RRP manipulator example
% please use meters and radians
T05n=subs(T05,{th1,d1,th2,a3,th4,d5},{pi/3,0.4,pi/4,0.2,pi/3,0.15})