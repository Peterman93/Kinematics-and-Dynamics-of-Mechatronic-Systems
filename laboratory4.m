%% Kinematics and Dynamics of Mechatronic Systems
% 
% 4th laboratory task
%
% _Piotr Bury_ , _Maciej Paczocha_

%% Draw the workspace
x=[0 0.02 0.04 0.06 0.08 0.1 0.12 0.14 0.16 0.18];
y=[0.95 0.97 0.99 1.01 1.03 1.05 1.07 1.09 1.11 1.13];
z=[0.75 0.77 0.79 0.81 0.83 0.85 0.87 0.89 0.91 0.93];
path=[x; y; z];
figure(1);
plot3(x,y,z,'--r')
for i=1:10
    joint4=path(:,i)'
    joint3=joint4-Pw'
    joint1=[T01_real(1,4) T01_real(2,4) T01_real(3,4)]
    pts34=[joint4; joint3];
    pts13=[joint3; joint1];
    pts01=[joint1; origin];
    line(pts34(:,1), pts34(:,2),pts34(:,3));
    line(pts13(:,1), pts13(:,2),pts13(:,3));
    line(pts01(:,1), pts01(:,2),pts01(:,3));
end