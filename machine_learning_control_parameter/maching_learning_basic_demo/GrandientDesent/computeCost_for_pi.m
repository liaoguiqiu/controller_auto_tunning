function J = computeCost_for_pi(para,theta)
	
	  J =[1  ,para(1) ,para(1)^2 ,para(1)* para(2) , para(2), para(2)^2 ] * theta  ;

end
