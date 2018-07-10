if (~exist('my_save'))
double_para_nanaly;
end

data_len = length(x);
Y = stable_err; %这里拟合二次函数  稳态时间
 %有 常数项 一次 二次 项
  X  =[ones(data_len,1)  ,x  ,x.*x ,x.*y, y, y.*y ]  ;
% X  =[ones(data_len,1)  ,initial_x  ,exp(initial_x.^4)   ]  ;
 
  %求解  
   max_iters  = 1000 ;% 迭代次数
   alpha =  1 ;  %学习 
   theta = zeros(length(X(1,:)),1);
%   theta = [ 1 ; 1 ; 1  ];
   theta5  = inv(X'*X)*X'*Y;                              %最小二乘求解
   cost_acheve  = computeCostMulti(X, Y, theta5);
%    [theta,J_history] = gradientDescent(X, Y, theta2, alpha, max_iters);
%    yout = X *theta;
   yout2 = X *theta5;
double_para_curve_fitting_plot;


 
  
  
