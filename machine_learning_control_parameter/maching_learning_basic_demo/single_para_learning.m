 load('my_save_vary_ki_only.mat');
 initial_x =  my_save.I';
 data_len = length(initial_x);
 Y = my_save.stable_time'; %������϶��κ���  ��̬ʱ��
 %�� ������ һ�� ���� ��
  X  =[ones(data_len,1)  ,initial_x  ,initial_x.*initial_x , initial_x.*initial_x.*initial_x    ]  ;
% X  =[ones(data_len,1)  ,initial_x  ,exp(initial_x.^4)   ]  ;
  figure (1)
  hold off ; 
  plot(X(:,2),Y,'r-+','MarkerSize', 4);  
 
  
  %���  
   max_iters  = 1000 ;% ��������
   alpha =  1 ;  %ѧϰ 
   theta = [ 80 ; -2900 ;  3.0000e+04   ];
%   theta = [ 1 ; 1 ; 1  ];
   theta2  = inv(X'*X)*X'*Y;                              %��С�������
   [theta,J_history] = gradientDescent(X, Y, theta2, alpha, max_iters);
   yout = X *theta;
   yout2 = X *theta2;
   figure(2)
   hold off;
   plot(Y,'rx','MarkerSize', 4);  hold on;
 
   plot(yout2);
   legend('initial', 'normal function')
   figure(3)
   hold off;
   plot(Y,'rx','MarkerSize', 4);  hold on;
   plot(yout);hold on ;
   plot(yout2);
   legend('initial','maching learing ', 'normal function')
  
   figure(4)
   plot(J_history);
   
  
