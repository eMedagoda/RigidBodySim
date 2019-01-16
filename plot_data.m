% plot MPC vehicle data

function plot_data(col)

M = csvread("build/output.csv");

figure(1)
subplot(3,1,1)
plot(M(:,1),M(:,2),col,'Linewidth',2)
hold on
title('u')
subplot(3,1,2)
plot(M(:,1),M(:,3),col,'Linewidth',2)
hold on
title('v')
subplot(3,1,3)
plot(M(:,1),M(:,4),col,'Linewidth',2)
hold on
title('w')

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
subplot(3,1,1)
plot(M(:,1),M(:,14)*180/pi,col,'Linewidth',2)
hold on
title('Lon')
subplot(3,1,2)
plot(M(:,1),M(:,15)*180/pi,col,'Linewidth',2)
hold on
title('Lat')
subplot(3,1,3)
plot(M(:,1),M(:,16)*180/pi,col,'Linewidth',2)
hold on
title('Alt')
