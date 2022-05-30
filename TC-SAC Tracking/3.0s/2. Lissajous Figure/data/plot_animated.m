clear all;
close all;

data=load('result_Liss.txt');
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
x_t(i,1)=50*sin(2*pi*1/3*0.075*t(2)*(i-1));
y_t(i,1)=50*cos(2*pi*1/2*0.075*t(2)*(i-1));

end
for i=1:size(t)
v=sqrt((50*sin(2*pi*1/3*0.075*(t(2)*(i-1)+0.001))-x_t(i,1))^2+(50*cos(2*pi*1/2*0.075*(t(2)*(i-1)+0.001))-y_t(i,1))^2)/0.001;

Cost(i,1)=(t(2)-t(1))*((x1(i)-x_t(i,1))*(x1(i)-x_t(i,1))+(x2(i)-y_t(i,1))*(x2(i)-y_t(i,1))+0.1*(x4(i)-v)*(x4(i)-v));
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
axis([-60 60 -60 60])
title(['Time: ' num2str(t(i),'%.2f') 's']);
hold off
subplot(9,1,8:9)
plot(t(1:i),Cost(1:i),'k','Linewidth',2)
axis([0 100 0 5*10^-2])
pause(0.001)
end
