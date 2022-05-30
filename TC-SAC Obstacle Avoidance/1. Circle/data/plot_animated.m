clear all;
close all;

data=load('result_Circle.txt');
dataSAC=load('planned_Trajectory.txt');
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
x_t(i,1)=50*sin(2*pi*0.031830988618379*t(2)*(i-1));
y_t(i,1)=50*cos(2*pi*0.031830988618379*t(2)*(i-1));
end
for i=1:size(t)
Cost(i,1)=(t(2)-t(1))*((x1(i)-x_t(i,1))*(x1(i)-x_t(i,1))+(x2(i)-y_t(i,1))*(x2(i)-y_t(i,1))+0.1*(x4(i)-10)*(x4(i)-10));
end
iter=1;
for i=0:0.001:1
 x_o(iter,1,1)=1*sin(2*pi*i)+50;
 y_o(iter,1,1)=1*cos(2*pi*i);
 
 x_o(iter,1,2)=1*sin(2*pi*i)-50;
 y_o(iter,1,2)=1*cos(2*pi*i);
 
 x_o(iter,1,3)=1*sin(2*pi*i);
 y_o(iter,1,3)=1*cos(2*pi*i)-50;
 iter=iter+1;
end
figure('units','normalized','outerposition',[0 0 1 1])
for i=1:size(t,1)
subplot(9,1,1:7)
plot(x_t,y_t,':b')
hold on
plot(x_t(i:i+50),y_t(i:i+50),'b','LineWidth',2)
hold on
plot(x1(1:i),x2(1:i),'k','Linewidth',2)
hold on
plot(dataSAC((i-1)*11+2,:),dataSAC((i-1)*11+3,:),'r','Linewidth',1)
    for l=1:3
        hold on
        plot(x_o(:,:,l),y_o(:,:,l),'r','LineWidth',2);
    end
axis([-60 60 -60 60])
title(['Time: ' num2str(t(i),'%.2f') 's']);
hold off
subplot(9,1,8:9)
plot(t(1:i),Cost(1:i),'k','Linewidth',2)
axis([0 100 0 2*10^-1])
pause(0.001)
end
