if (~exist('my_save'))
triple_para_analyze;
end

data_len = length(x);
Y = total_err; %������϶��κ���  ��̬ʱ��
 %�� ������ һ�� ���� ��
%  X  =[ones(data_len,1) ,x,y,z,x.^2 ,y.^2, z.^2,x.^3 ,y.^3,z.^3,...
%       x.*y,x.*z,z.*y,x.^2.*y,x.^2.*z,y.^2.*x,y.^2.*z,z.^2.*x,z.^2.*y ,x.*y.*z  ];
 X  =[ones(data_len,1) ,x,y,z,x.^2 ,y.^2, z.^2,...
      x.*y,x.*z,z.*y];
  %���  
   max_iters  = 1000 ;% ��������
   alpha =  1 ;  %ѧϰ 
   theta = zeros(length(X(1,:)),1);
%   theta = [ 1 ; 1 ; 1  ];
   theta4  = inv(X'*X)*X'*Y;                              %��С�������
      cost_stable = computeCostMulti(X, Y, theta4);
%    [theta,J_history] = gradientDescent(X, Y, theta2, alpha, max_iters);
%    yout = X *theta;
   yout2 = X *theta4;
   
  
 
  triple_para_curve_fitting_plot;


 
  
  
