function [para, J_history] = gradientDescent_for_pi_step(para, theta, alpha )
% X is m*(n+1) matrix 
% y is m*1
% para  PIDto tuning
% theta is the den for the costfunction
% alpha is a number  which  determine the learn speed 
% num_iters is limit number of iterators
	
        para1 = para(1) - alpha*( theta(2) + 2 * para(1)*theta(3) + para(2)*theta(4));
        para2 = para(2) - alpha*( theta(5) + 2 * para(2)*theta(6) + para(1)*theta(4));
        para(1) = para1;
        para(2) = para2;  
        J_history = computeCost_for_pi(para, theta);
        %这里只需要theta 和PI来进行目价值判断
	
  
end