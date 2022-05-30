clear all;
close all;

data=load('result_Liss.txt');

t=data(:,1);
x1=data(:,2);
x2=data(:,3);
x3=data(:,4);
x4=data(:,5);
x5=data(:,6);
x6=data(:,7);
x7=data(:,8);
Cost=data(:,9);

u1=data(:,10);
u2=data(:,11);

controller=data(:,12);
for i=1:size(t)+50
x_t(i,1)=50*sin(2*pi*1/3*0.075*t(2)*(i-1));
y_t(i,1)=50*cos(2*pi*1/2*0.075*t(2)*(i-1));

end
for i=1:size(t)
v=sqrt((50*sin(2*pi*1/3*0.075*(t(2)*(i-1)+0.001))-x_t(i,1))^2+(50*cos(2*pi*1/2*0.075*(t(2)*(i-1)+0.001))-y_t(i,1))^2)/0.001;

% Cost(i,1)=(t(2)-t(1))*((x1(i)-x_t(i,1))*(x1(i)-x_t(i,1))+(x2(i)-y_t(i,1))*(x2(i)-y_t(i,1))+0.1*(x4(i)-v)*(x4(i)-v));
end


iter=1;
for i=0:0.001:1
 x_o(iter,1,1)=1*sin(2*pi*i)+43;
 y_o(iter,1,1)=1*cos(2*pi*i);
 
 x_o(iter,1,2)=1*sin(2*pi*i)-43;
 y_o(iter,1,2)=1*cos(2*pi*i);
 
 x_o(iter,1,3)=1*sin(2*pi*i);
 y_o(iter,1,3)=1*cos(2*pi*i)-50;
  
 x_o(iter,1,5)=1*sin(2*pi*i)+48;
 y_o(iter,1,5)=1*cos(2*pi*i)-47;
 
 x_o(iter,1,6)=1*sin(2*pi*i)-48;
 y_o(iter,1,6)=1*cos(2*pi*i)+47;
 
 x_o(iter,1,7)=1*sin(2*pi*i)+48;
 y_o(iter,1,7)=1*cos(2*pi*i)+47;
 
 x_o(iter,1,8)=1*sin(2*pi*i);
 y_o(iter,1,8)=1*cos(2*pi*i);
 iter=iter+1;
end

u1=data(:,10);
u2=data(:,11);

controller=data(:,12);

figure('Name','Static Obstacle Avoidance','units','normalized')%,'outerposition',[0 0 0.9905 1])
subplot(21,1,1:13)
plot(x1,x2,x_t(1:size(t)),y_t(1:size(t)),'LineWidth',2)
    for l=1:8
        hold on
        plot(x_o(:,:,l),y_o(:,:,l),'r','LineWidth',2);
    end
xlabel('x');
ylabel('y');
title('x-y-Plot');
ytickformat('%g m')
xtickformat('%g m')
xt = get(gca, 'XTick');
% set(gca, 'FontSize', 20)
% set(gca, 'Position', [0 0 10 10])
legend('TC-SAC','Reference Trajectory');


subplot(21,1,18:21)
plot(t,Cost,'LineWidth',2)
xlabel('t [s]');
ylabel('Cost');
title('Cost');
xt = get(gca, 'XTick');
% set(gca, 'Position', [0 11 10 10])
% axesHandles = findobj(get(figureHandle,'Children'), 'flat','Type','axes');
% axis(axesHandles,'square')

% set(gca, 'FontSize', 20)


% figure(2)
% title('States');
% subplot(8,1,1)
% plot(t,x1,t,x_t(1:size(t)),'LineWidth',2)
% xlabel('t [s]');
% ylabel('x [m]');
% title('x-Coordinate');
% legend('TC-SAC','Reference Trajectory');
% 
% subplot(8,1,2)
% plot(t,x2,t,y_t(1:size(t)),'LineWidth',2)
% xlabel('t [s]');
% ylabel('y [m]');
% title('y-Coordinate');
% legend('TC-SAC','Reference Trajectory');
% 
% subplot(8,1,3)
% plot(t,x3,'LineWidth',2)
% xlabel('t [s]');
% ylabel('\Psi [\circ]');
% title('Orientation');
% 
% subplot(8,1,4)
% plot(t,x4,'LineWidth',2)
% xlabel('t [s]');
% ylabel('v [m/s]');
% title('velocity');
% 
% subplot(8,1,5)
% plot(t,x5,'LineWidth',2)
% xlabel('t [s]');
% ylabel('\alpha [\circ]');
% title('side slip angle');
% 
% subplot(8,1,6)
% plot(t,x6,'LineWidth',2)
% xlabel('t [s]');
% ylabel('\Psi [\circ]');
% title('change in orientation');
% 
% subplot(8,1,7)
% plot(t,x7,'LineWidth',2)
% xlabel('t [s]');
% ylabel('\delta [\circ]');
% title('steering angle');
% 
% subplot(8,1,8)
% plot(t,Cost,'LineWidth',2)
% xlabel('t [s]');
% ylabel('Cost');
% title('Cost');
% 
% figure(3)
% title('Control Inputs');
% 
% subplot(3,1,1)
% plot(t,u1,'LineWidth',2)
% xlabel('t [s]');
% ylabel('u_\delta');
% title('steering input');
% 
% subplot(3,1,2)
% plot(t,u2,'LineWidth',2)
% xlabel('t [s]');
% ylabel('M [Nm]');
% title('Torque');
% 
% subplot(3,1,3)
% plot(t,controller,'LineWidth',2)
% xlabel('t[s]');
% ylabel({'BH[0]','SAC[1]'});
% title('Controller');
