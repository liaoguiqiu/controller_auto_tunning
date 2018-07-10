clear all; 
% ���ڲ������ݼ���ֻ��KP�仯��
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

%---------�洢���г�ʼ��--------%
my_save = [];
my_save.acheive_time  =  0; %���ڱ���
my_save.stable_time   =  0;
my_save.overshot    = 0 ;
my_save.stable_err = 0;
my_save.total_err = 0; 
my_save.total_enegy = 0; 
my_save.P  =  0; %���ڱ���
my_save.I   =  0;
my_save.D    = 0 ;
my_save.para = paramters.value;
save_point = 1;  %����asv��ָ��
save_best_point = 1 ;
my_save.bestpara =paramters.value;
my_save.bestcost = 0;
my_save.mincost_fater_all_loop = 1000;
all_loop_cnt =1 ;
min_cost  =  10000;


while(1)
     for para_it= 1:1:paramters.num
         for   para=paramters.min(para_it) : paramters.step(para_it): paramters.max(para_it)  %kp�仯ѭ��
           paramters.value(para_it) = para ;
           ctrl_loop; 
%            ctrl_loop_plot;
         end
        paramters.value = my_save.bestpara(length(my_save.bestpara(:,1)),:);   
    end
    
    
    
    % save mincost after all loop
    all_loop_cnt= all_loop_cnt + 1;
    my_save.mincost_fater_all_loop(all_loop_cnt) = min_cost;
    if(abs( my_save.mincost_fater_all_loop(all_loop_cnt) - ...
            my_save.mincost_fater_all_loop(all_loop_cnt-1))<0.1)
        break;
    end
end
%��ʾ��Ѳ���ֵ
  ctrl_loop; 
  ctrl_loop_plot;
my_save.bestpara(length(my_save.bestpara(:,1)),:)
 
save('my_save_best_para.mat','my_save');

%%����ֵ����
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

