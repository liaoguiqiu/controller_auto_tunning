 function f = double_para_function (x )
    theta =  [101.8849 -251.8506  261.7145   40.6405   12.5115   20.8227];
    f =   [ 1 ,x(1)   ,x(1).*x(1)  ,x(1).*x(2),x(2),x(2).*x(2)] * theta'  ;
 end