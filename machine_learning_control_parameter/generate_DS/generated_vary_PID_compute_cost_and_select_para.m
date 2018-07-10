% 用于产生数据集：只有KP变化的
clear all; 
ts=0.04;%采样时间
%系统模型
sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1,0]);%速度模型（输入角度，输出速度）
dsys=c2d(sys,ts,'z');                          %连续模型离散
[num,den]=tfdata(dsys,'v');                    %获取分子分母
x=[0,0,0]';                                  %P、I、D的保存
x2_1=0;

kp_var_step = 0.001;  %参数调节步长
kp_max      = 0.1 ;    %最大参数
kp_min      = 0.01 ; %最小参数

ki_var_step = 0.003;  %参数调节步长
ki_max      = 0.1 ;    %最大参数
ki_min      = 0 ; %最小参数

kd_var_step = 0.00002;  %参数调节步长
kd_max      =  0.001 ;    %最大参数
kd_min      = -0.001 ;    %最小参数

kp=0.03;
ki=1.74189585656528e-09;     
kd= 0.000008321786918629;
u_1=0.0;u_2=0.0;u_3=0.0;          %系统参数初始化                 
y_1=0;y_2=0;y_3=0;                %系统参数初始化
error_1=0;  
acheive_time = 100 ;

stable_bufferlen = 10;
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
save_point = 1;  %保存asv的指针
save_best_point = 1 ;
my_save.bestpara =[ 0 ; 0 ;0] ;
 min_cost  =  10000;
 for i_p=1:1:((kp_max - kp_min)/kp_var_step)  %kp变化循环
      kp=kp_min + i_p* kp_var_step;
     for i_i=1:1:((ki_max - ki_min)/ki_var_step)  %ki变化循环
      ki=ki_min + i_i* ki_var_step;
      for  i_d = 1:1 :((kd_max - kd_min)/kd_var_step)  %kd变化循环
          kd = kd_min + i_d * kd_var_step;
          
       u_1=0.0;u_2=0.0;u_3=0.0;          %系统参数初始化                 
       y_1=0;y_2=0;y_3=0;                %系统参数初始化
       error_1=0; 
       
       
       %------特征变量初始化------%
       acheive_time = 100 ;    %首次到达时间
       stable_time  = 100;     %稳态时间
       over_shot = -100 ;
       stable_err = 100;
       total_err = 10000;
       find_achive =0;
       find_stable  = 0;
       
       
        for k=1:1:100
                time(k)=k*ts;                    %时间序列     
                rin(k)=100;                      %目标阶跃值
                % rin(k)=1000*sin(k/100*pi);   
                % u(k)=ABS_LIMIT(kp*x(1) ,5)+kd*x(2)+ ABS_LIMIT( ki*x(3),5); % 计算控制量
                u(k)= kp*x(1)+kd*x(2)+  ki*x(3); % 计算控制量
                u(k)  = ABS_LIMIT(u(k),10);     %限幅
                % -----------离散系统转移方程-------------------%
                yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+num(4)*u_3;
                u_3=u_2;u_2=u_1;u_1=u(k);
                y_3=y_2;y_2=y_1;y_1=yout(k);
                %----------Return of PID parameters------------%
                error(k)=rin(k)-yout(k);       
                x(1)=error(k);                % Calculating P
                x2_1=x(2);
                x(2)=(error(k)-error_1)/ts;   % Calculating D
                x(3)=x(3)+ABS_LIMIT( error(k),1)*ts;        % Calculating I
                x(3)=ABS_LIMIT( x(3),1);   
                error_1=error(k);
                
             %         将 error_buffer 循环移位
                stable_buffer = [ error(k),stable_buffer(1:stable_bufferlen-1)];
                averange = sum(stable_buffer)/stable_bufferlen;  %误差均值
                stable_rmse  = (stable_buffer-averange)*(stable_buffer-averange)';   %均方误差
                %记录响应时间
                
                if(find_achive==0&& abs(error(k)) <  ( 0.08*rin(k)))
              %        if(abs(error(k)) <  (0.02*rin(k)))            
                    acheive_time = k ;
                    find_achive = 1 ;
                end
                if(over_shot < (-error(k))) %记录超调
                    over_shot  = - error(k) ;
                end
                %记录到达稳态时间
                  if(find_stable ==0&& ( stable_rmse <  (stable_bufferlen*0.00002*rin(k))) && ( abs(error(k)) < (0.01*rin(k))))
                    stable_time = k ;
                    stable_err = -stable_buffer(1);
                    find_stable = 1;  
                    total_err =sum( abs(error))/ length(error);
                    break;
                end  
        end %控制调剂循环
        
        if(stable_err < (0.01*rin(k)) ) %只保存稳态误差小的参数
           %                     保存到asv
                   my_save.P (save_point) = kp ;
                   my_save.I (save_point) = ki ;
                   my_save.D (save_point) = kd ;
                   my_save.acheive_time(save_point) = acheive_time ;
                   my_save.stable_time(save_point) = stable_time ;
                   my_save.overshot(save_point) = over_shot ;
                   my_save.stable_err(save_point) = stable_err;
                   my_save.total_err (save_point) = total_err;
                   save_point  = save_point +1; 
                   % is cost minner
                   % cost function : stable_time/30  + over_shot /100 + total_err/100 ; 
                   this_cost = stable_time/30  + over_shot /100 + total_err/100 ; 
                   if(this_cost < min_cost ) 
                    my_save.bestpara(:,save_best_point) = [ kp ; ki ;kd] ; 
                    min_cost  = this_cost;
                    save_best_point  = save_best_point+1;  
                   end
        end      
%         figure(1);  % k控制结束后的画图
%         hold off;
%         plot(yout,'r' ); hold on;
%         plot( rin ,'g');hold on;
%         plot( rin+over_shot ,'b');hold on;
%         plot( rin+ stable_err,'y');hold on;
%         plot(acheive_time,yout(acheive_time),'r*'); hold on;
%         plot(stable_time,yout(stable_time),'b*'); hold on;
%         legend('yout','aim','overshot','stable','achieve_time','stable_time');
%         clear yout  error; 


      end  % kd变化循环
     end   %ki变化循环
 end  % kp变化循环      
%显示最佳参数值
my_save.bestpara(:,length(my_save.bestpara(1,:)))
 
save('my_save_vary_PID.mat','my_save');

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

