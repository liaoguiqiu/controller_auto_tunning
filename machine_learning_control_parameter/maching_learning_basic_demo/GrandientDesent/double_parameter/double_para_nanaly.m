clear all; 
load('my_save_vary_PD_only.mat' );

x=  my_save.P' ; %  rescal the vector
y=  my_save.D' ;%  rescal the vector
Initial_max_x  = max(x);
Initial_max_y  = max(y);
x = x /Initial_max_x ; 
y = y /Initial_max_y ; 

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
 
 
 stable_time = my_save.stable_time';
 acheve_time = my_save.acheive_time';
 over_shot= my_save.overshot';
  total_err = my_save.total_err';
 stable_err = my_save.stable_err';
[X,Y]=meshgrid(minx:(maxx-minx)/100:maxx,miny:(maxy-miny)/100:maxy);

figure(2)

%----%
subplot(5,2,1)
hold off;
Z=griddata(x,y,stable_time,X,Y);
mesh(X,Y,Z);
xlabel('x');
ylabel('y');
zlabel('stable_time');
subplot(5,2,2)
hold off
plot3(x,y,stable_time,'.')
xlabel('x');
ylabel('y');
zlabel('stable_time');
%----%
subplot(5,2,3)
hold off;
Z=griddata(x,y,over_shot,X,Y);
mesh(X,Y,Z);
xlabel('x');
ylabel('y');
zlabel('over_shot');
subplot(5,2,4)
hold off
plot3(x,y,over_shot,'.')
xlabel('x');
ylabel('y');
zlabel('over_shot');
%----%
subplot(5,2,5)
hold off;
Z=griddata(x,y,acheve_time,X,Y);
mesh(X,Y,Z);
xlabel('x');
ylabel('y');
zlabel('acheve_time');
subplot(5,2,6)
hold off
plot3(x,y,acheve_time,'.')
xlabel('x');
ylabel('y');
zlabel('acheve_time');
 %----%
 subplot(5,2,7);hold off;
Z=griddata(x,y,total_err,X,Y);
mesh(X,Y,Z);xlabel('x');ylabel('y');
zlabel('total_err');
subplot(5,2,8);hold off
plot3(x,y,total_err,'.');xlabel('x');ylabel('y');zlabel('total_err');
  %----%
 subplot(5,2,9);hold off;
Z=griddata(x,y,stable_err,X,Y);
mesh(X,Y,Z);xlabel('x');ylabel('y');
zlabel('stable_err');
subplot(5,2,10);hold off
plot3(x,y,stable_err,'.');xlabel('x');ylabel('y');zlabel('stable_err');
 
% 
% my_save.x(629858)
% my_save.y(629858)
% my_save.D(629858)
  
  
