function [para, J_history] = gradientDescent_for_pid_step(para, theta, alpha )
% X is m*(n+1) matrix 
% y is m*1
% para  PIDto tuning
% theta is the den for the costfunction
% alpha is a number  which  determine the learn speed 
% num_iters is limit number of iterators
	%diff test
    
    syms  x1 x2 x3   ; 
   f = [1 ,x1,x2,x3,x1^2 ,x2^2, x3^2,...
      x1*x2,x1*x3,x3*x2] * theta;
%     f =  [1 ,x1,x2,x3,x1.^2 ,x2.^2, x3.^2,x1.^3 ,x2.^3,x3.^3,...
%       x1.*x2,x1.*x3,x3.*x2,x1.^2.*x2,x1.^2.*x3,x2.^2.*x1,x2.^2.*x3,x3.^2.*x1,x3.^2.*x2 ,x1.*x2.*x3  ] * theta;
    %  diff function  
     f1 = diff(f,x1);
     f2 = diff(f,x2);
     f3 = diff(f,x3);
     %  caculated diff value
    x1 = para(1);  x2 = para(2);  x3 = para(1);
    diff1  =subs(f1);
    diff2 = subs(f2) ;
    diff3 = subs(f3);
        para1 = para(1) - alpha*diff1;
        para2 = para(2) - alpha*diff2;
        para3 = para(3) - alpha*diff3;
        para(1) = para1;
        para(2) = para2;  
        para(3) = para3;
        J_history = computeCost_for_pid(para, theta);
        %这里只需要theta 和PI来进行目价值判断
	
  
end