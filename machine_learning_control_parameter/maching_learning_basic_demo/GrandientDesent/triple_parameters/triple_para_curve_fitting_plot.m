 figure(3)
 hold off;
  subplot(2,2,1)
 scatter3(x,y,z, [],Y/max(Y),'filled');
 xlabel('x');
ylabel('y');
zlabel('z');
title(' initail') ;
 subplot(2,2,2)
 scatter3(x,y,z, [],yout2/max(yout2),'filled');
 xlabel('x');
ylabel('y');
zlabel('z');
title(' fit') ;
