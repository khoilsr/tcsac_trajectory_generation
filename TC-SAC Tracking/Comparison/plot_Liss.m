clear all;
close all;

data05=load('result_Liss05.txt');

t05=data05(:,1);
x105=data05(:,2);
x205=data05(:,3);
x305=data05(:,4);
x405=data05(:,5);
x505=data05(:,6);
x605=data05(:,7);
x705=data05(:,8);
Cost05=data05(:,9);

for i=1:size(t05)+50
x_t05(i,1)=50*sin(2*pi*1/3*0.075*t05(2)*(i-1));
y_t05(i,1)=50*cos(2*pi*1/2*0.075*t05(2)*(i-1));

end
for i=1:size(t05)
v05=sqrt((50*sin(2*pi*1/3*0.075*(t05(2)*(i-1)+0.001))-x_t05(i,1))^2+(50*cos(2*pi*1/2*0.075*(t05(2)*(i-1)+0.001))-y_t05(i,1))^2)/0.001;

Cost05(i,1)=(t05(2)-t05(1))*((x105(i)-x_t05(i,1))*(x105(i)-x_t05(i,1))+(x205(i)-y_t05(i,1))*(x205(i)-y_t05(i,1))+0.1*(x405(i)-v05)*(x405(i)-v05));
end

u105=data05(:,10);
u205=data05(:,11);

controller05=data05(:,12);

data10=load('result_Liss10.txt');

t10=data10(:,1);
x110=data10(:,2);
x210=data10(:,3);
x310=data10(:,4);
x410=data10(:,5);
x510=data10(:,6);
x610=data10(:,7);
x710=data10(:,8);
Cost10=data10(:,9);

for i=1:size(t10)+50
x_t10(i,1)=50*sin(2*pi*1/3*0.075*t10(2)*(i-1));
y_t10(i,1)=50*cos(2*pi*1/2*0.075*t10(2)*(i-1));

end
for i=1:size(t10)
v10=sqrt((50*sin(2*pi*1/3*0.075*(t10(2)*(i-1)+0.001))-x_t10(i,1))^2+(50*cos(2*pi*1/2*0.075*(t10(2)*(i-1)+0.001))-y_t10(i,1))^2)/0.001;

Cost10(i,1)=(t10(2)-t10(1))*((x110(i)-x_t10(i,1))*(x110(i)-x_t10(i,1))+(x210(i)-y_t10(i,1))*(x210(i)-y_t10(i,1))+0.1*(x410(i)-v10)*(x410(i)-v10));
end

u110=data10(:,10);
u210=data10(:,11);

controller10=data10(:,12);

data30=load('result_Liss30.txt');

t30=data30(:,1);
x130=data30(:,2);
x230=data30(:,3);
x330=data30(:,4);
x430=data30(:,5);
x530=data30(:,6);
x630=data30(:,7);
x730=data30(:,8);
Cost30=data30(:,9);

for i=1:size(t30)+50
x_t30(i,1)=50*sin(2*pi*1/3*0.075*t30(2)*(i-1));
y_t30(i,1)=50*cos(2*pi*1/2*0.075*t30(2)*(i-1));

end
for i=1:size(t30)
v30=sqrt((50*sin(2*pi*1/3*0.075*(t30(2)*(i-1)+0.001))-x_t30(i,1))^2+(50*cos(2*pi*1/2*0.075*(t30(2)*(i-1)+0.001))-y_t30(i,1))^2)/0.001;

Cost30(i,1)=(t30(2)-t30(1))*((x130(i)-x_t30(i,1))*(x130(i)-x_t30(i,1))+(x230(i)-y_t30(i,1))*(x230(i)-y_t30(i,1))+0.1*(x430(i)-v30)*(x430(i)-v30));
end

u130=data30(:,10);
u230=data30(:,11);

controller30=data30(:,12);

figure(1)
subplot(21,1,1:13)

plot(x105,x205,x110,x210,x130,x230,x_t05(1:size(t05)),y_t05(1:size(t05)),'LineWidth',2)
xlabel('x');
ylabel('y');
title('x-y-Plot');
legend('T_{Pre}:0.5s','T_{Pre}:1.0s','T_{Pre}:3.0s','Reference Trajectory');

subplot(21,1,18:21)
plot(t05,Cost05,t10,Cost10,t30,Cost30,'LineWidth',2)
xlabel('t [s]');
ylabel('Cost');
title('Cost');

figure(2)
title('States');
subplot(8,1,1)
plot(t05,x105,t10,x110,t30,x130,t05,x_t05(1:size(t05)),'LineWidth',2)
xlabel('t [s]');
ylabel('x [m]');
title('x-Coordinate');
legend('0.5s','1.0s','3.0s','Reference Trajectory');

subplot(8,1,2)
plot(t05,x205,t10,x210,t30,x230,t05,y_t05(1:size(t05)),'LineWidth',2)
xlabel('t [s]');
ylabel('y [m]');
title('y-Coordinate');
legend('0.5s','1.0s','3.0s','Reference Trajectory');

subplot(8,1,3)
plot(t05,x305,t10,x310,t30,x330,'LineWidth',2)
xlabel('t [s]');
ylabel('\Psi [\circ]');
title('Orientation');
legend('0.5s','1.0s','3.0s');

subplot(8,1,4)
plot(t05,x405,t10,x410,t30,x430,'LineWidth',2)
xlabel('t [s]');
ylabel('v [m/s]');
title('velocity');
legend('0.5s','1.0s','3.0s');

subplot(8,1,5)
plot(t05,x505,t10,x510,t30,x530,'LineWidth',2)
xlabel('t [s]');
ylabel('\alpha [\circ]');
title('side slip angle');
legend('0.5s','1.0s','3.0s');

subplot(8,1,6)
plot(t05,x605,t10,x610,t30,x630,'LineWidth',2)
xlabel('t [s]');
ylabel('\Psi [\circ]');
title('change in orientation');
legend('0.5s','1.0s','3.0s');

subplot(8,1,7)
plot(t05,x705,t10,x710,t30,x730,'LineWidth',2)
xlabel('t [s]');
ylabel('\delta [\circ]');
title('steering angle');
legend('0.5s','1.0s','3.0s');

subplot(8,1,8)
plot(t05,Cost05,t10,Cost10,t30,Cost30,'LineWidth',2)
xlabel('t [s]');
ylabel('Cost');
title('Cost');
legend('0.5s','1.0s','3.0s');

figure(3)
title('Control Inputs');

subplot(3,1,1)
plot(t05,u105,t10,u110,t30,u130,'LineWidth',2)
xlabel('t [s]');
ylabel('u_\delta');
title('steering input');
legend('0.5s','1.0s','3.0s');

subplot(3,1,2)
plot(t05,u205,t10,u210,t30,u230,'LineWidth',2)
xlabel('t [s]');
ylabel('M [Nm]');
title('Torque');
legend('0.5s','1.0s','3.0s');

subplot(3,1,3)
plot(t05,controller05,t10,controller10,t30,controller30,'LineWidth',2)
xlabel('t[s]');
ylabel({'BH[0]','SAC[1]'});
title('Controller');
legend('0.5s','1.0s','3.0s');