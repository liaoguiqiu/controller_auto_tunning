

  clear yout  error u rin; 
%--observe error and control error---%
observe_err =  0.01 ; 
control_err  = 0;


loop.kpOuter = paramters.value(1);                              % ??PID??
loop.kiOuter = paramters.value(2);
loop.kdOuter = paramters.value(3);

loop.kpInner = paramters.value(4);                             % ??PID??
loop.kiInner = paramters.value(5);
loop.kdInner = paramters.value(6);
loop.highThres = paramters.value(7);
loop.velThres = paramters.value(8);
loop.fulloutVel = paramters.value(9);
loop.fullout = paramters.value(10);
% loop.kpOuter = 0.4;                              % ??PID??
% loop.kiOuter = 0;
% loop.kdOuter = 11;
%  
% loop.kpInner = 0.04;                             % ??PID??
% loop.kiInner = 0.004;
% loop.kdInner = 0.35;

%=============================control parameter init========================%
motorOutOld = zeros(1, LEN);
statusOld = zeros(1, LEN);  
loop.errorOuter = zeros(1,10);                   % ??? ?1??????10?????
loop.intSumOuter = 0;                            % ???
loop.diffOuter = zeros(1, 10);                   % ???
loop.statusOuter = 0;
loop.errorInner = zeros(1,10);                   % ????1??????10?????
loop.intSumInner = [ 0; 0];                            % ???
loop.diffInner = zeros(1, 10);                   % ???
loop.statusInner = 0;
loop.basicThr = 0;
loop.actualHigh =0 ;
%==============================????????=========================%
%-out_save_initail--%
clear out_save;
out_save.real_v =0;
out_save.real_p= 0;
out_save.ob_v =0;
out_save.ob_p= 0;
out_save.u =0;
out_save.u_ki =0; 
       %------特征变量初始化------%
       acheive_time = timeCnt ;    %首次到达时间
       stable_time  = timeCnt;     %稳态时间
       over_shot = -100 ;
       stable_err = 100;
       total_err = 10000;
       total_enegy = 10000;
       find_achive =0;
       find_stable  = 0;
for k=1:1:timeCnt
             i=k;
time(k)=k*ts;                    %时间序列     
                 %目标阶跃值

%-------- control loop of jinjing start----%
 loop.aimHigh(i) = 5000;                       % ?????
 rin(k)= loop.aimHigh(i) ;   
    %--------?????----------%
    loop = Controller(loop, i);
    
    %--------??????????--------%
    [loop.actualVel, motorOutOld, statusOld] = discreate_function(loop.outInner(i)+ control_err, numUse ,denUse, motorOutOld, statusOld);
    loop.actualHigh = loop.actualHigh + loop.actualVel * 0.04; 
    loop.statusInner = loop.actualVel*(1+ observe_err*rand(1) -0.5* observe_err);   
    loop.statusOuter = loop.actualHigh*(1+ observe_err*rand(1) -0.5* observe_err);
    
 
    error(i) = loop.aimHigh(i) - loop.actualHigh ;
    
 %----- control loop of jinjing end-------%   

% rin(k)=1000*sin(k/100*pi);   
% u(k)=ABS_LIMIT(kp*x(1) ,5)+kd*x(2)+ ABS_LIMIT( ki*x(3),5); % 计算控制量
u(k)=loop.outInner(i); % 计算控制量
% -----------离散系统转移方程-------------------%
 yout(k)  = loop.actualHigh ;
  %--save_out_data-%
 out_save.real_v(k) = loop.actualVel ;
 out_save.real_p(k) = loop.actualHigh ;
 out_save.ob_v(k) = loop.statusInner ;
 out_save.ob_p(k) = loop.statusOuter ;
 out_save.u(k)   = u(k) ;
 out_save.u_ki(k)   = loop.intSumInner(1);
 
%----------Return of PID parameters------------%
 error(k)=rin(k)-yout(k);                
             %         将 error_buffer 循环移位
                stable_buffer = [ error(k),stable_buffer(1:stable_bufferlen-1)];
                averange = sum(stable_buffer)/stable_bufferlen;  %误差均值
                stable_rmse  = (stable_buffer-averange)*(stable_buffer-averange)';   %均方误差
                %记录响应时间
                
                if(find_achive==0&& abs(error(k)) <   100)
              %        if(abs(error(k)) <  (0.02*rin(k)))            
                    acheive_time = k ;
                    find_achive = 1 ;
                end
                if(over_shot < (-error(k))) %记录超调
                    over_shot  = - error(k) ;
                end
                %记录到达稳态时间
                    if(k>stable_bufferlen&&  find_stable ==0&& ( stable_rmse <  (stable_bufferlen*25)) && ( abs(error(k)) < 100))
                     stable_time = k-stable_bufferlen ;
                    stable_err = -stable_buffer(1);
                    find_stable = 1;  
                    total_err =sum( abs(error));
                    total_enegy = sum(u);
                    break;
                end  
        end %控制循环
        
         %--save some paraeters after the control loop---%
        if(stable_err < (0.01*rin(k)) ) %只保存稳态误差小的参数
           %                     保存到asv
                   
                   my_save.para(save_point,:) =  paramters.value ; 
                   my_save.acheive_time(save_point) = acheive_time ;
                   my_save.stable_time(save_point) = stable_time ;
                   my_save.overshot(save_point) = over_shot ;
                   my_save.stable_err(save_point) = stable_err;
                   my_save.total_err (save_point) = total_err;
                   my_save.total_enegy(save_point) = total_enegy;
                   save_point  = save_point +1; 
                   % is cost minner
                   % cost function : stable_time/30  + over_shot /100 + total_err/100 ; 
                  this_cost = stable_time/350 +acheive_time/200  +...
                      over_shot/150  + total_err/3.4580e+05 + total_enegy/2.2348e+04 ; 
%                  this_cost = stable_time + acheive_time  ; 
                   if(this_cost < min_cost ) 
                    my_save.bestpara(save_best_point,:) = paramters.value ; 
                    
                    min_cost  = this_cost;
                    my_save.bestcost(save_best_point ) = min_cost ; 
                    save_best_point  = save_best_point+1;  
                   end
        end  
        
     
        