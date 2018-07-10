clear all; 
load('my_save_vary_PID.mat' );

x=  my_save.P' ; %  rescal the vector
y=  my_save.I' ;%  rescal the vector
z=  my_save.D' ;%  rescal the vector
Initial_max_x  = max(x);
Initial_max_y  = max(y);
Initial_max_z  = max(z);
%normallize
x = x /Initial_max_x ; 
y = y /Initial_max_y ; 
z = z /Initial_max_z ; 

figure(1)
hold off;
plot(my_save.acheive_time);hold on;
plot(my_save.overshot);hold on ;
plot(my_save.stable_time);hold on;
plot(my_save.total_err);hold on;
plot(my_save.stable_err);hold on;
legend('achive-time','over-shot','stable-time','totalerr','stab-err');
 
 maxx = max(x); minx = min(x);
 maxy = max(y) ;miny= min(y);
  maxz = max(z) ;minz= min(z);
 
 
 stable_time = my_save.stable_time';
 acheve_time = my_save.acheive_time';
 over_shot= my_save.overshot';
  total_err = my_save.total_err';
 stable_err = my_save.stable_err';
%  
%  [X,Y,Z]=meshgrid(minx:(maxx-minx)/100:maxx,miny:(maxy-miny)/100:maxy,minz:(maxz-minz)/100:maxz);
%  
 figure(2)
 hold off;
 subplot(3,2,1)
 scatter3(x,y,z, [],stable_time/max(stable_time),'filled');
 xlabel('x');
ylabel('y');
zlabel('z');
title('stable_time') ;

 subplot(3,2,2)
 scatter3(x,y,z, [],acheve_time/max(acheve_time),'filled');
 xlabel('x');
ylabel('y');
zlabel('z');
title('acheve_time') ;
subplot(3,2,3)
 scatter3(x,y,z, [],over_shot/max(over_shot),'filled');
 xlabel('x');
ylabel('y');
zlabel('z');
title('over_shot') ;

subplot(3,2,4)
 scatter3(x,y,z, [],total_err/max(total_err),'filled');
 xlabel('x');
ylabel('y');
zlabel('z');
title('total_err') ;

subplot(3,2,5)
 scatter3(x,y,z, [],stable_err/max(stable_err),'filled');
 xlabel('x');
ylabel('y');
zlabel('z');
title('stable_err') ;
 
