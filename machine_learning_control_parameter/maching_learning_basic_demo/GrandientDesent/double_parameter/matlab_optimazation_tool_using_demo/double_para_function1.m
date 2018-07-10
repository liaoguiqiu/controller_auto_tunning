
%原函数降维
function f = double_para_function1 (x )
    
    y = -0.7928;  %首次设置的初值
    theta =  [101.8849 -251.8506  261.7145   40.6405   12.5115   20.8227];
    f =   [ 1 ,x   ,x.*x  ,x.*y,y,y.*y] * theta'  ;
 end