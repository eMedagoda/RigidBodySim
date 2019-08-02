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
plot(M(:,1),M(:,52)*180/pi,col,'Linewidth',2)
hold on
plot(M(:,1),M(:,8)*180/pi,'r','Linewidth',2)
title('\phi')
subplot(3,1,2)
plot(M(:,1),M(:,53)*180/pi,col,'Linewidth',2)
hold on
plot(M(:,1),M(:,9)*180/pi,'r','Linewidth',2)
title('\theta')
subplot(3,1,3)
plot(M(:,1),M(:,54)*180/pi,col,'Linewidth',2)
hold on
plot(M(:,1),M(:,10)*180/pi,'r','Linewidth',2)
title('\psi')

figure(4)
subplot(3,1,1)
plot(M(:,1),M(:,29),'g','Linewidth',2)
hold on
plot(M(:,1),M(:,36),col,'Linewidth',2)
plot(M(:,1),M(:,11),'r','Linewidth',2)
title('X')
subplot(3,1,2)
plot(M(:,1),M(:,30),'g','Linewidth',2)
hold on
plot(M(:,1),M(:,37),col,'Linewidth',2)
plot(M(:,1),M(:,12),'r','Linewidth',2)
title('Y')
subplot(3,1,3)
plot(M(:,1),M(:,31),'g','Linewidth',2)
hold on
plot(M(:,1),M(:,38),col,'Linewidth',2)
plot(M(:,1),M(:,13),'r','Linewidth',2)
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
plot(M(:,1),M(:,46),col,'Linewidth',2)
hold on
title('b_{ax}')
subplot(3,2,4)
plot(M(:,1),M(:,47),col,'Linewidth',2)
hold on
title('b_{ay}')
subplot(3,2,6)
plot(M(:,1),M(:,48),col,'Linewidth',2)
hold on
title('b_{az}')

figure(6)
subplot(3,2,1)
plot(M(:,1),M(:,23)*180/pi,col,'Linewidth',2)
hold on
title('g_{x}')
subplot(3,2,3)
plot(M(:,1),M(:,24)*180/pi,col,'Linewidth',2)
hold on
title('g_{y}')
subplot(3,2,5)
plot(M(:,1),M(:,25)*180/pi,col,'Linewidth',2)
hold on
title('g_{z}')
subplot(3,2,2)
plot(M(:,1),M(:,49)*180/pi,col,'Linewidth',2)
hold on
title('b_{gx}')
subplot(3,2,4)
plot(M(:,1),M(:,50)*180/pi,col,'Linewidth',2)
hold on
title('b_{gy}')
subplot(3,2,6)
plot(M(:,1),M(:,51)*180/pi,col,'Linewidth',2)
hold on
title('b_{gz}')

figure(7)
title('GPS')
subplot(3,2,1)
plot(M(:,1),M(:,36),col,'Linewidth',2)
title('N')
subplot(3,2,3)
plot(M(:,1),M(:,37),col,'Linewidth',2)
title('E')
subplot(3,2,5)
plot(M(:,1),M(:,38),col,'Linewidth',2)
title('D')
subplot(3,2,2)
plot(M(:,1),M(:,39),col,'Linewidth',2)
title('V_{n}')
subplot(3,2,4)
plot(M(:,1),M(:,40),col,'Linewidth',2)
title('V_{e}')
subplot(3,2,6)
plot(M(:,1),M(:,41),col,'Linewidth',2)
title('V_{d}')