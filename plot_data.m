% plot MPC vehicle data

function plot_data(col)

M = csvread("build/output.csv");

figure(1)
subplot(4,1,1)
plot(M(:,1),M(:,2),col,'Linewidth',2)
hold on
title('u')
subplot(4,1,2)
plot(M(:,1),M(:,3),col,'Linewidth',2)
hold on
title('v')
subplot(4,1,3)
plot(M(:,1),M(:,4),col,'Linewidth',2)
hold on
title('w')
subplot(4,1,4)
plot(M(:,1),sqrt(M(:,2).*M(:,2) + M(:,3).*M(:,3) + M(:,4).*M(:,4)),col,'Linewidth',2)
hold on
title('V')

figure(2)
subplot(3,1,1)
plot(M(:,1),M(:,5)*180/pi,col,'Linewidth',2)
hold on
title('p')
subplot(3,1,2)
plot(M(:,1),M(:,6)*180/pi,col,'Linewidth',2)
hold on
title('q')
subplot(3,1,3)
plot(M(:,1),M(:,7)*180/pi,col,'Linewidth',2)
hold on
title('r')

figure(3)
subplot(3,1,1)
plot(M(:,1),M(:,8)*180/pi,col,'Linewidth',2)
hold on
title('\phi')
subplot(3,1,2)
plot(M(:,1),M(:,9)*180/pi,col,'Linewidth',2)
hold on
title('\theta')
subplot(3,1,3)
plot(M(:,1),M(:,10)*180/pi,col,'Linewidth',2)
hold on
title('\psi')

figure(4)
subplot(3,1,1)
plot(M(:,1),M(:,11),col,'Linewidth',2)
hold on
title('X')
subplot(3,1,2)
plot(M(:,1),M(:,12),col,'Linewidth',2)
hold on
title('Y')
subplot(3,1,3)
plot(M(:,1),M(:,13),col,'Linewidth',2)
hold on
title('Z')

figure(5)
subplot(3,2,1)
plot(M(:,1),M(:,20),col,'Linewidth',2)
hold on
title('a_{x}')
subplot(3,2,3)
plot(M(:,1),M(:,21),col,'Linewidth',2)
hold on
title('a_{y}')
subplot(3,2,5)
plot(M(:,1),M(:,22),col,'Linewidth',2)
hold on
title('a_{z}')
subplot(3,2,2)
plot(M(:,1),M(:,23)*180/pi,col,'Linewidth',2)
hold on
title('g_{p}')
subplot(3,2,4)
plot(M(:,1),M(:,24)*180/pi,col,'Linewidth',2)
hold on
title('g_{q}')
subplot(3,2,6)
plot(M(:,1),M(:,25)*180/pi,col,'Linewidth',2)
hold on
title('g_{r}')

figure(6)
subplot(3,2,1)
plot(M(:,1),M(:,29),col,'Linewidth',2)
hold on
title('N')
subplot(3,2,3)
plot(M(:,1),M(:,30),col,'Linewidth',2)
hold on
title('E')
subplot(3,2,5)
plot(M(:,1),M(:,31),col,'Linewidth',2)
hold on
title('D')
subplot(3,2,2)
plot(M(:,1),M(:,32)*180/pi,col,'Linewidth',2)
hold on
title('Vn')
subplot(3,2,4)
plot(M(:,1),M(:,33)*180/pi,col,'Linewidth',2)
hold on
title('Ve')
subplot(3,2,6)
plot(M(:,1),M(:,34)*180/pi,col,'Linewidth',2)
hold on
title('Vd')