clear all; 
% 用于产生数据集：只有KP变化的
%--import the sys idented --%
 clc;
if (~(exist('numUse') && exist('denUse')))
      load('model_data_7');
    %--------????????--------%
    num = tf1.Numerator;
    den = tf1.Denominator;
    ts = 0.04;                                  % ????
    sys = tf(num, den);                         % ??????
    dsys = c2d(sys, ts, 'z');                   % ?????
    [numUse, denUse] = tfdata(dsys, 'v');
end
%--import the sys idented end --%
timeCnt = 1000;                                 % ???? time = timeCnt * ts 
LEN = length(denUse) - 1;                       % ??????
% the  parameter initial value
%  0.3000         0    1.0000    0.0100    0.0040    1.0000

paramters = [];
paramters.num = 10;  % number of parameter
% intial value:pid of 2 loop(6) ;threshold of 2 loop (2); full out of 2
% loop(2)
paramters.value = [0.5,  0 , 11  ,  0.1 , 0.01 , 0.12 ,300,400,500,50];   
paramters.max =  [ 0.5 , 1   , 35 ,  0.1  , 0.01  ,   2 ,1000,500,2000,50 ] ;
paramters.min =  [ 0.3 , 0   , 1   ,  0.01 , 0.001 ,   1  ,300,100,400,30 ] ;
% paramters.min  =  paramters.value;
paramters.step= [ 0.1 , 0.5 , 10 ,0.01  , 0.001  , 0.1 ,100,100,100,10 ] ;
paramters.it  =   [ 0 , 0 , 0 ,0  , 0 , 0 ,0,0,0,0 ] ;
  

acheive_time = 100 ;
stable_bufferlen = 10/ts;   
stable_buffer=zeros(1,stable_bufferlen);

%---------存储队列初始化--------%
my_save = [];
my_save.acheive_time  =  0; %用于保存
my_save.stable_time   =  0;
my_save.overshot    = 0 ;
my_save.stable_err = 0;
my_save.total_err = 0; 
my_save.total_enegy = 0; 
my_save.P  =  0; %用于保存
my_save.I   =  0;
my_save.D    = 0 ;
my_save.para = paramters.value;
save_point = 1;  %保存asv的指针
save_best_point = 1 ;
my_save.bestpara =paramters.value;
my_save.bestcost = 0;
my_save.mincost_fater_all_loop = 1000;
all_loop_cnt =1 ;
min_cost  =  10000;


 for   para1=paramters.min(1) : paramters.step(1): paramters.max(1)  %kp变化循环
   paramters.value(1) = para1 ;
   for   para2=paramters.min(2) : paramters.step(2): paramters.max(2)  %kp变化循环
     paramters.value(2) = para2 ;     
      for   para3=paramters.min(3) : paramters.step(3): paramters.max(3)  %kp变化循环
          paramters.value(3) = para3 ;  
            for   para4=paramters.min(4) : paramters.step(4): paramters.max(4)  %kp变化循环
            paramters.value(4) = para4 ;     
                  for   para5=paramters.min(5) : paramters.step(5): paramters.max(5)  %kp变化循环
                    paramters.value(5) = para5 ;  
                     for   para6=paramters.min(6) : paramters.step(6): paramters.max(6)  %kp变化循环
                     paramters.value(6) = para6 ;
                      for   para7=paramters.min(7) : paramters.step(7): paramters.max(7)   
                      paramters.value(7) = para7 ; 
                          for   para8=paramters.min(8) : paramters.step(8): paramters.max(8)   
                          paramters.value(8) = para8 ; 
                           for   para9=paramters.min(9) : paramters.step(9): paramters.max(9)   
                           paramters.value(9) = para9 ;
                               for   para10=paramters.min(10) : paramters.step(10): paramters.max(10)   
                               paramters.value(10) = para10 ; 
                                ctrl_loop; 
                                ctrl_loop_plot;
                               end % para9   
                           end % para9   
                          end % para8   
                      end % para7     
                     end % para6
                  end % para5
            end % para4
      end % para3
   end % para2
 end % para1
   paramters.value = my_save.bestpara(length(my_save.bestpara(:,1)),:); 
  
%显示最佳参数值
  ctrl_loop; 
  ctrl_loop_plot;
my_save.bestpara(length(my_save.bestpara(:,1)),:)
 
save('my_save_best_para.mat','my_save');

%%绝对值函数
function out=ABS_LIMIT(x,limit)
 if(x<(-limit))
     out = -limit;
 elseif(x>limit)
     out = limit;
 else
     out = x;
 end
return;
end

