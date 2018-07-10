function [para, J_history] = gradientDescent_for_pi_step2(para, theta, alpha )
% X is m*(n+1) matrix 
% y is m*1
% para  PIDto tuning
% theta is the den for the costfunction
% alpha is a number  which  determine the learn speed 
% num_iters is limit number of iterators
	
        syms  x1 x2    ; 
    f = [1  ,x1 , x1^2 ,x1* x2 , x2, x2^2 ] * theta;
    %  diff function  
     f1 = diff(f,x1);
     f2 = diff(f,x2);
    
     %  caculated diff value
    x1 = para(1);  x2 = para(2);  
    diff1  =subs(f1);
    diff2 = subs(f2) ;
   
        para1 = para(1) - alpha*diff1;
        para2 = para(2) - alpha*diff2;
       
        para(1) = para1;
        para(2) = para2;  
      
          
        J_history = computeCost_for_pi(para, theta);
        %这里只需要theta 和PI来进行目价值判断
	
  
end