function [theta, J_history] = gradientDescent(X, y, theta, alpha, num_iters)
% X is m*(n+1) matrix 
% y is m*1
% theta is (n+1)*1 matrix
% alpha is a number 
% num_iters is number of iterators

	
	m = length(y); % number of training examples
    n = length(theta);  % number of parmerters to learn 
	J_history = zeros(num_iters, 1);  %cost function的值的变化过程
	%预先定义了迭代的次数
    temp = zeros(1,n)
	for iter = 1:num_iters
        for the_i=1:1:n
		temp(the_i) = theta(the_i) - (alpha / m) * sum((X * theta - y).* X(:,the_i));	 
        end
         for the_i=1:1:n
		theta(the_i) = temp(the_i);
        end
		J_history(iter) = computeCost(X, y, theta);

	end

end