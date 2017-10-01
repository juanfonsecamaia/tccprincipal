figure(2)
hold on
realcart = horzcat(xr,yr,zr);
for i=1:1:length(qreal)-1

 robot.plot(qreal(i,:));
 hold on
  plot2(realcart(i:i+1,:),'r');
 end
robot.plot(qreal(length(qreal),:));
plot3(rx,ry,rz);