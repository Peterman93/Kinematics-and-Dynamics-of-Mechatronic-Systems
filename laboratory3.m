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

%
% multiplication of matrices to obtain HTM
T01=A1;
T05=A1*A2*A3*A4*A5;

%% Determining the constants and random values of joint coordinates in the ranges
% input constants
d1_input = 0.4;
d5_input = 0.15;

% Drawing the workspace on the 1st figure
figure(1);
hold on
% for th1_input=-pi/2:pi/8:pi/2%18
%    for th2_input=-pi/4:pi/4:pi/4 %8
%        for a3_input=0.2:0.1:0.6 
%            for th4_input=-pi/2:pi/2:pi/2 %4
%                T05_r=subs(T05,{th1,d1,th2,a3,th4,d5},{th1_input,d1_input, th2_input, a3_input, th4_input, d5_input});
%                P_r = [T05_r(1,4); T05_r(2,4); T05_r(3,4)];
%                plot3(P_r(1,1),P_r(2,1),P_r(3,1),'.')
%            end
%        end
%    end
% end

%% Determine the paths
%Determine the 1st path
x1=[0.39 0.39 0.29 0.23 0.21 0.19  0.17  0.15 0.14 0.12];
y1=[0.46 0.44 0.40 0.30 0.20 0.10 -0.10 -0.2 -0.3 -0.3];
z1=[0.03 0.29 0.43 0.43 0.37 0.37  0.35  0.18 0.15 0.02];
path1=[x1; y1; z1];

%Determine the 2nd path
x2=[0.00  0.00 0.00 0.14 0.24 0.24 0.32 0.27  0.03  0.00];
y2=[0.45  0.45 0.38 0.35 0.24 0.00 -0.2 -0.27 -0.42 -0.53];
z2=[-0.05 0.24 0.57 0.78 0.70 0.60 0.4  0.3   0.2   -0.13];
path2=[x2; y2; z2];

%% Draw both paths on the 1st figure
plot3(x1,y1,z1,'--r')
%hold on
plot3(x2,y2,z2,'--g')
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
hold off

%% 1st path: Generating motion path for each joint
figure(2)

hold on
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
%% 
% Drawing motion path for each joint
plot3(x1,y1,z1,'--r');
for i=1:10
    joint4=path1(:,i);   
    %calculation of q1,q2,q3,q4 with inverse kinematics
    th1_inverse=atan2(joint4(2,1),joint4(1,1));
    th2_inverse=atan2((joint4(3,1)-d1_input),sqrt(joint4(1,1)^2+joint4(2,1)^2));
    a3_inverse= sqrt(joint4(1,1)^2+joint4(2,1)^2+(joint4(3,1)-d1_input)^2);
    th4_inverse = pi/2-(pi)*rand; %not possible to select specific range for q4, so that it will be generated randomly
    %determining the matrix for points belonging to path
    T01_real=subs(T01,{th1,d1},{th1_inverse,d1_input});
    T05_real=subs(T05,{th1,d1,th2,a3,th4,d5},{th1_inverse,d1_input, th2_inverse, a3_inverse, th4_inverse, d5_input});
    Pw = [T05_real(1,3);T05_real(2,3);T05_real(3,3)]*d5_input;    
    joint3=joint4-Pw;
    joint1=[T01_real(1,4); T01_real(2,4); T01_real(3,4)];
    pts34=[joint4'; joint3'];
    pts13=[joint3'; joint1'];
    pts01=[joint1'; [0 0 0]];
    line(pts34(:,1), pts34(:,2),pts34(:,3));
    line(pts13(:,1), pts13(:,2),pts13(:,3));
    line(pts01(:,1), pts01(:,2),pts01(:,3));    
end
hold off


%% 2nd path: Generating motion path for each joint
figure(3)
hold on
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
CartesianCoord = []
%% 
% Drawing motion path for each joint
plot3(x2,y2,z2,'--g');
for i=1:10
    joint4=path2(:,i);   
    %calculation of q1,q2,q3,q4 with inverse kinematics
    th1_inverse=atan2(joint4(2,1),joint4(1,1));
    th2_inverse=atan2((joint4(3,1)-d1_input),sqrt(joint4(1,1)^2+joint4(2,1)^2));
    a3_inverse= sqrt(joint4(1,1)^2+joint4(2,1)^2+(joint4(3,1)-d1_input)^2);
    th4_inverse = pi/2-(pi)*rand; %not possible to select specific range for q4, so that it will be generated randomly
    %determining the matrix for points belonging to path
    T01_real=subs(T01,{th1,d1},{th1_inverse,d1_input});
    T05_real=subs(T05,{th1,d1,th2,a3,th4,d5},{th1_inverse,d1_input, th2_inverse, a3_inverse, th4_inverse, d5_input});
    Pw = [T05_real(1,3);T05_real(2,3);T05_real(3,3)]*d5_input;    
    joint3=joint4-Pw;
    joint1=[T01_real(1,4); T01_real(2,4); T01_real(3,4)];
    pts34=[joint4'; joint3'];
    pts13=[joint3'; joint1'];
    pts01=[joint1'; [0 0 0]];
    line(pts34(:,1), pts34(:,2),pts34(:,3));
    line(pts13(:,1), pts13(:,2),pts13(:,3));
    line(pts01(:,1), pts01(:,2),pts01(:,3));    
end
hold off





