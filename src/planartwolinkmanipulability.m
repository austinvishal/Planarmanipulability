%% manipulability and force ellipsoid of a RR planar robot
close all;
clear all;
clc;
l1=2;
l2=2;
% th1=0; th2=pi/4;
% x=2;
 y=0;
 th=[0,pi/4,pi/2,3*pi/4,pi];
 set(gcf, 'color', 'white','units','pixels','position',[0 0 1920 1080]);

 vidObj=VideoWriter('planarmanipulability.mp4');
 vidObj.FrameRate = 24;
 open(vidObj);
 for i=1:length(th)
%      th1=0;
%      th2=th(i)
%     th1=pi/2;
    th1=pi/4;
     th2=th(i)-pi/2;
%  for x=0:0.2:4
% case 3 from singular
% x=2;
% y=0.2;
% case 2 near singular
% for x=-2:0.2:4
%     case 1
% x=2;
% y=2;
% for x=0:0.2:4
%% IK
% [th1,th2]= IK_RR(x,y,l1,l2)
%% jacbian matrix
J= [-l1*sin(th1)-l2*sin(th1+th2), -l2*sin(th1+th2);
    l1*cos(th1)+l2*cos(th1+th2), l2*cos(th1+th2);];

%% eigen values and vectors
[V,D]= eig(J*J');  % eigen value
ev=eig(J*J'); % eigen vector

t1= atan2(V(2,2),V(1,2)); % finding the rotation angle, the principal axis is line of eigen vector
xe= sqrt(max(abs(ev))); % principal axis length xe,ye
ye= sqrt(min(abs(ev)));
%% plot pose manipulator configuration using forward kinematics to check
x=l1*cos(th1)+l2*cos(th1+th2);
y=l1*sin(th1)+l2*sin(th1+th2)
plot(0,0, 'ko','MarkerFaceColor','k','MarkerSize', 8)
hold on
plot([0, l1*cos(th1),x],[0, l1*sin(th1),y],'k-o',...
    'linewidth',1.5,'MarkerFaceColor','r',...
    'MarkerSize',5) % plot pose manipulator configuration

%% Velocity or manipulability ellipsoid
aa= [cos(t1),-sin(t1);sin(t1),cos(t1)]*[xe*cosd(0:360)/xe; ye*sind(0:360)/xe]; % scaled version
plot(x+aa(1,:),y+aa(2,:),'k-')
text(aa(1)+2,aa(2)+2,'mu_v','fontsize',14,'Interpreter', 'latex');
hold on

%% force ellipsoid
bb=[cos(t1),-sin(t1);sin(t1),cos(t1)]*[ye*sind(0:360)/xe; xe*cosd(0:360)/xe]; % scaled version
plot(x+bb(1,:),y+bb(2,:),'r-')
text(bb(1)+3,bb(2)+2,'mu_f','fontsize',14,'Interpreter', 'latex');
axis([-1.5*(l1+l2) 1.5*(l1+l2) -1.5*(l1+l2) 1.5*(l1+l2)])
axis square
grid on
 hold off
pause(0.025)
     currFrame= getframe(gcf);
            writeVideo(vidObj, currFrame);
 end
     close(gcf)
 close(vidObj);