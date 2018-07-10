
function J = computeCostMulti(X, y, theta)

	m = length(y); % number of training examples
	J = 0;
	predictions = X * theta;
	J = 1/(2*m)*(predictions - y)' * (predictions - y);

end

