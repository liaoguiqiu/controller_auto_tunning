function J = computeCost_for_pid(para,theta)
	

%        J =  [1 ,para(1),para(2) ,para(3) ,para(1).^2 ,para(2) .^2, para(3) .^2,para(1).^3 ,para(2) .^3,para(3) .^3,...
%       para(1).*para(2) ,para(1).*para(3) ,para(3) .*para(2) ,para(1).^2.*para(2) ,para(1).^2.*para(3) ,para(2) .^2.*para(1),para(2) .^2.*para(3) ,para(3) .^2.*para(1),para(3) .^2.*para(2)  ,para(1).*para(2) .*para(3)   ]* theta  ;
% 	   
   J = [1 ,para(1),para(2),para(3),para(1)^2 ,para(2)^2, para(3)^2,...
      para(1)*para(2),para(1)*para(3),para(3)*para(2)] * theta;
end
