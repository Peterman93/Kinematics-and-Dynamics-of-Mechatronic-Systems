%% Kinematics and Dynamics of Mechatronic Systems
% 3rd laboratory task
%
% _Piotr Bury_ , _Maciej Paczocha_

%% Determine a Homogeneous Transformation matrix
%
clear all; clc

% declaration of symbols
syms th1 d1 th2 a3 th4 d5 q1 q2 q3
% Determination of a symbolic form of HT matrices – application of mA function
A1=mA(th1,d1,0,sym(pi/2));
A2=mA(th2,0,0,0);
A3=mA(0,0,a3,0);
A4=mA(th4,0,0,sym(pi/2));
A5=mA(sym(pi/2),d5,0,0);

%% multiplication of matrices to obtain the position of the links
T01=A1;
T05=A1*A2*A3*A4*A5;

%% Determining the constants and random values of joint coordinates in the ranges
% input constants
d1_input = 0.4;
d5_input = 0.15;

%% Subtitute random generated ranges into the matrices
%T01_real=subs(T01,{th1,d1},{th1_input,d1_input});
% Pw = [T05_real(1,3);T05_real(2,3);T05_real(3,3)]*d5_input;
% Pa = P_real-Pw;

figure(1);
hold on
for th1_input=-pi/2:pi/8:pi/2%18
   for th2_input=-pi/4:pi/4:pi/4 %8
       for a3_input=0.2:0.1:0.6 
           for th4_input=-pi/2:pi/2:pi/2 %4
               T05_r=subs(T05,{th1,d1,th2,a3,th4,d5},{th1_input,d1_input, th2_input, a3_input, th4_input, d5_input});
               P_r = [T05_r(1,4); T05_r(2,4); T05_r(3,4)];
               plot3(P_r(1,1),P_r(2,1),P_r(3,1),'.')
           end
       end
   end
end

%% Draw the 1st path
x1=[0.39 0.39 0.29 0.23 0.21 0.19  0.17  0.15 0.14 0.12];
y1=[0.46 0.44 0.40 0.30 0.20 0.10 -0.10 -0.2 -0.3 -0.3];
z1=[0.03 0.29 0.43 0.43 0.37 0.37  0.35  0.18 0.15 0.02];
path1=[x1; y1; z1];


x2=[0.00  0.00 0.00 0.14 0.24 0.24 0.32 0.27  0.03  0.00];
y2=[0.45  0.45 0.38 0.35 0.24 0.00 -0.2 -0.27 -0.42 -0.53];
z2=[-0.05 0.24 0.57 0.78 0.70 0.60 0.4  0.3   0.2   -0.13];
path2=[x2; y2; z2];

figure(2);
plot3(x1,y1,z1,'--r')
%hold on
plot3(x2,y2,z2,'--g')
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
hold off
% for i=1:10
%     joint4=path(:,i)'
%     joint3=joint4-Pw'
%     joint1=[T01_real(1,4) T01_real(2,4) T01_real(3,4)]
%     pts34=[joint4; joint3];
%     pts13=[joint3; joint1];
%     pts01=[joint1; origin];
%     line(pts34(:,1), pts34(:,2),pts34(:,3));
%     line(pts13(:,1), pts13(:,2),pts13(:,3));
%     line(pts01(:,1), pts01(:,2),pts01(:,3));
% end



% %% Defining origin and constant position of the 1st link
% origin = [0 0 0];
% joint1 = [T01(1,4) T01(2,4) T01(3,4)]; 
% % Position of joint1 remains constant as it's only dependent from d1 constant variable
% % joint2 position cannot be distinguished as it wouldn't be visible in the
% % workspace visualization
% 




