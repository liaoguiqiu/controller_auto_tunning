if (~exist('theta1'))
    clear all;
    close all;
 triple_fit_acheve_time ;
 triple_fit_overshot ;
 triple_fit_stableerr  ;
 triple_fit_stabletime;
 triple_fit_totalerr ;
end
 %有 常数项 一次 二次 项
%  X  = [ones(data_len,1) ,x,y,z,x.^2 ,y.^2, z.^2,x.^3 ,y.^3,z.^3,...
%       x.*y,x.*z,z.*y,x.^2.*y,x.^2.*z,y.^2.*x,y.^2.*z,z.^2.*x,z.^2.*y ,x.*y.*z  ]; 
 X  =[ones(data_len,1) ,x,y,z,x.^2 ,y.^2, z.^2,...
      x.*y,x.*z,z.*y];
% cost =  X *theta2  +  X *theta3  +  X *theta4 ;

 
theta_fuse= (theta1 );
 
cost =  X *theta_fuse;

%%%%

figure(3)
hold off;
% plot3(x,y,cost,'b');hold on
 scatter3(x,y,z, [],cost/max(cost),'filled');   hold on;
 xlabel('x');
ylabel('y');
zlabel('z');
title(' fit') ;
   max_iters  = 100000 ;% 迭代次?
   alpha =  0.0001 ;  %学习 
   para = [0.2;0.2;0.2 ];
%   [para, J_history] = gradientDescent_for_pi(para, theta_fuse, alpha,max_iters);
    limit = 0.0000005;
    J_history = 0; 
    Gradien_history  = 0;
    %初值限幅
    para(1)= LIMIT(para(1),minx,maxx);
    para(2)= LIMIT(para(2),miny,maxy);
    para(3)= LIMIT(para(3),minz,maxz);
       
	for iter = 1:max_iters
   
    [para, J ] = gradientDescent_for_pid_step(para, theta_fuse, alpha );     
         J_history(iter) = J ;    
               if(iter>1)
                gradent  = abs( J_history(iter) - J_history(iter-1));
                Gradien_history(iter) = gradent;
%                 if( para(1)<minp||para(1)>maxp || para(2)<mini || para(2)>maxi  )
%                     break;
%                 end
                if(gradent<limit)
                    break;
                end
               end
          figure(3);  
       
         plot3(para(1),para(2),para(3),'r*','MarkerSize',16);hold on;
         figure(4);
         hold off ;
         plot(J_history,'r');
         figure(5);
         hold off ;
         plot(Gradien_history,'r');
         pause(0.02);  % pause to show the figure 
    end
    learning_result(1) = para(1)*Initial_max_x
    learning_result(2) = para(2)*Initial_max_y
    learning_result(3) = para(3)*Initial_max_z
    clear   J_history   

