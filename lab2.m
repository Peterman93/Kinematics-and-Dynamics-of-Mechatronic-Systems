% This script determines a Homogeneous Transformation matrix
clear all
% declaration of symbols
syms th1 d1 th2 a3 th4 d5 q1 q2 q3
% determination of a symbolic form of HT matrices –
% application of mA function
A1=mA(th1,d1,0,sym(pi/2));
A2=mA(th2,0,0,0);
A3=mA(0,0,a3,0);
A4=mA(th4,0,0,sym(pi/2));
A5=mA(sym(pi/2),d5,0,0);

% multiplication of matrices
T03=A1*A2*A3;
T05=A1*A2*A3*A4*A5
%%input constants
d1_input = 1;
d5_input = 2;

%%input variables
th1_input = pi/2-pi*rand
th2_input = pi/4-(pi/2)*rand
a3_input=0.6-rand*0.4
th4_input = pi/2-(pi)*rand;
T05_real=subs(T05,{th1,d1,th2,a3,th4,d5},{th1_input,d1_input, th2_input, a3_input, th4_input, d5_input})

%%End effector coordinates from random joint variables
Px_real = T05_real(1,4)
Py_real = T05_real(2,4);
Pz_real = T05_real(3,4);
P_real = [Px_real; Py_real; Pz_real];
Pw = [T05_real(1,3);T05_real(2,3);T05_real(3,3)]*d5_input
Pa = P_real-Pw

th1_inverse=atan2(Pa(2,1),Pa(1,1))
th2_inverse=atan2((Pa(3,1)-d1_input),sqrt(Pa(1,1)^2+Pa(2,1)^2))
a3_inverse= sqrt(Pa(1,1)^2+Pa(2,1)^2+(Pa(3,1)-d1_input)^2)