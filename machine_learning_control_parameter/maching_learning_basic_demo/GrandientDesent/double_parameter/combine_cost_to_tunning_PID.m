if (~exist('theta1'))
    clear all;
    close all;
 double_para_curve_fitting_acheve_time ;
 double_para_curve_fitting_stable_time ;
 double_para_curve_fitting_over_shot  ;
 double_para_curve_fitting_total_err;
 double_para_curve_fitting_stable_err ;
end
 %有 常数项 一次 二次 项
 X  =[ones(data_len,1)  ,x  ,x.*x ,x.*y, y, y.*y ]  ;
 
% cost =  X *theta2  +  X *theta3  +  X *theta4 ;
theta_fuse= (theta1 );
cost =  X *theta_fuse;
figure(3)
hold off;
plot3(x,y,cost,'b');hold on
   max_iters  = 100000 ;% 迭代次数
   alpha =  0.0001 ;  %学习 
   para = [0.02;0.0 ];
%   [para, J_history] = gradientDescent_for_pi(para, theta_fuse, alpha,max_iters);
    limit = 0.0000001;
    J_history = 0; 
    Gradien_history  = 0;
    %初值限幅
    para(1)= LIMIT(para(1),minx,maxx);
    para(2)= LIMIT(para(2),miny,maxy);
       
	for iter = 1:max_iters
        
     [para, J ] = gradientDescent_for_pi_step(para, theta_fuse, alpha ); 
%      [para, J ] = gradientDescent_for_pi_step2(para, theta_fuse, alpha );     
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
         plot3(para(1),para(2),J,'r*');hold on;
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
    
    clear   J_history   

