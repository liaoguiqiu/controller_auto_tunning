clear all; 
% 用于产生数据集：只有KP变化的
%--import the sys idented --%
 clc;
if (~(exist('numUse') && exist('denUse')))
    load('Model_Data');
    %--------????????--------%
    ts = 0.04;                                  % ????
    sys = tf(num, den);                         % ??????
    dsys = c2d(sys, ts, 'z');                   % ?????
    [numUse, denUse] = tfdata(dsys, 'v');
end
%--import the sys idented end --%
timeCnt = 1000;                                 % ???? time = timeCnt * ts 
LEN = length(denUse) - 1;                       % ??????

paramters = [];
paramters.num = 6;  % number of parameter
paramters.value = [0.4,  0 , 12  ,  0.03 , 0.005 , 0.35 ]; 
paramters.max =  [ 0.5 , 1   , 35 ,  0.1  , 0.01  ,   1  ] ;
paramters.min =  [ 0.3 , 0   , 1   ,  0.01 , 0.001 ,  0.1  ] ;
% paramters.min  =  paramters.value;
paramters.step= [ 0.1 , 0.5 , 10 ,0.01  , 0.001  , 0.01  ] ;
paramters.it  =   [ 0 , 0 , 0 ,0  , 0 , 0  ] ;
  

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
my_save.P  =  0; %用于保存
my_save.I   =  0;
my_save.D    = 0 ;
my_save.para = paramters.value;
save_point = 1;  %保存asv的指针
save_best_point = 1 ;
my_save.bestpara =paramters.value;
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
                     ctrl_loop;
                     end % para6
                  end % para5
            end % para4
      end % para3
   end % para2
 end % para1
       
%显示最佳参数值
my_save.bestpara(length(my_save.bestpara(:,1)),:)
 
save('my_save_vary_all_para.mat','my_save');

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

