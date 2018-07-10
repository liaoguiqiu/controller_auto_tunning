function [para, J_history] = gradientDescent_for_pi(para, theta, alpha,num_iters)
% X is m*(n+1) matrix 
% y is m*1
% para  PIDto tuning
% theta is the den for the costfunction
% alpha is a number  which  determine the learn speed 
% num_iters is limit number of iterators
 
	J_history = zeros(num_iters, 1);  %cost function的值的变化过程
	%预先定义了迭代的次数
  
    limit = 0.1;
	for iter = 1:num_iters
        
        P = para(1) - alpha*( theta(2) + 2 * para(1)*theta(3) + para(2)*theta(4));
        I = para(2) - alpha*( theta(5) + 2 * para(2)*theta(6) + para(1)*theta(4));
        para(1) = P;
        para(2) = I;
       
        %这里只需要theta 和PI来进行目价值判断
		J_history(iter) = computeCost_for_pi(para, theta);
   if(iter>1)
	gradent  = abs( J_history(iter) - J_history(iter-1));
    if(gradent<limit)
        break;
    end
   end

end