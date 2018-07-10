%  PID Controller 采用PID控制器控制已经辨识的系统
 clc;
   clear yout  error u rin; 
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
timeCnt = 1000;                                 % ???? time = timeCnt * ts 

LEN = length(denUse) - 1;                       % ??????
motorOutOld = zeros(1, LEN);
statusOld = zeros(1, LEN);  

%--observe error and control error---%
observe_err =  0.01 ; 
control_err  = 0;
%-out_save_initail--%
out_save.real_v =0;
out_save.real_p= 0;
out_save.ob_v =0;
out_save.ob_p= 0;
out_save.u =0;
out_save.u_ki =0; 

%=============================????????========================%
 

loop.errorOuter = zeros(1,10);                   % ??? ?1??????10?????
loop.intSumOuter = 0;                            % ???
loop.diffOuter = zeros(1, 10);                   % ???
loop.statusOuter = 0;

loop.errorInner = zeros(1,10);                   % ????1??????10?????
loop.intSumInner = [ 0; 0];                            % ???
loop.diffInner = zeros(1, 10);                   % ???
loop.statusInner = 0;
 
loop.kpOuter = 0.5;                              % ??PID??
loop.kiOuter = 0;
loop.kdOuter = 11;
 
loop.kpInner = 0.1;                             % ??PID??
loop.kiInner = 0.01;
loop.kdInner = 0.12;
loop.highThres = 300;
loop.velThres = 400;
loop.fulloutVel = 1300;
loop.fullout = 50;
loop.basicThr = 0;
loop.actualHigh =0 ;
if (exist('my_save') )
paramters.value = my_save.bestpara(length(my_save.bestpara(:,1)),:); 
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
end

%==============================????????=========================%

 
%first in and output 
ctrl_u_old= zeros(1,8);
ctrl_out_old= zeros(1,8);
u_1=0.0;u_2=0.0;u_3=0.0;          %系统参数初始化                 
y_1=0;y_2=0;y_3=0;                %系统参数初始化??
error_1=0;  
       acheive_time = timeCnt ;    %首次到达时间
       stable_time  = timeCnt;     %稳态时间
       over_shot = 0 ;
       find_achive =0;
       find_stable  = 0;
       stable_bufferlen = 10/ts; %  10 sec
       stable_err =0;
stable_buffer=zeros(1,stable_bufferlen);
pid_struct.olderr = zeros(1,10);
 
pid_struct.I = 0;
pid_struct.out = 0;
for k=1:1:1000
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
    
 
%     error(i) = loop.aimHigh(i) - loop.actualHigh(i);
    
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
 
    
                %         将 error_buffer 循环移位
                error(k)=rin(k)-yout(k); 
                stable_buffer = [ error(k),stable_buffer(1:stable_bufferlen-1)];
                averange = sum(stable_buffer)/stable_bufferlen;  %误差均值
                stable_rmse  = (stable_buffer-averange)*(stable_buffer-averange)';   %均方误差
                %记录响应时间
                
                if(find_achive==0&& abs(error(k)) <  ( 0.03*rin(k)))
              %        if(abs(error(k)) <  (0.02*rin(k)))            
                    acheive_time = k ;
                    find_achive = 1 ;
                end
                if(over_shot < (-error(k))) %记录超调
                   over_shot  = - error(k) ;
                end
                %记录到达稳态时间, judge after some time
                 if(k>stable_bufferlen&&  find_stable ==0&& ( stable_rmse <  (stable_bufferlen*25)) && ( abs(error(k)) < 100))
                    stable_time = k -stable_bufferlen;
                    stable_err = -stable_buffer(1);
                    find_stable = 1;  
                    total_err =sum( abs(error)) ;
                   
                    total_enegy = sum(abs(u)); %                   
                      break;
                end  
end
        figure(1);  % k控制结束后的画图
        %--position track-%
        subplot(2,1,1)
        hold off;
         plot(out_save.ob_p ); hold on;
        plot(yout,'r' ); hold on;
        plot( rin ,'g');hold on;
        plot( rin+over_shot ,'b');hold on;
        plot( rin+ stable_err,'y');hold on;
        plot(acheive_time,yout(acheive_time),'r*'); hold on;
        plot(stable_time,yout(stable_time),'b*'); hold on;
        legend('observe','yout','aim','overshot','stable','achieve_time','stable_time');
         %--velocity track-%
        subplot(2,1,2)
        hold off;
        plot(  loop.outOuter, 'b' ); hold on;
           plot(   out_save.ob_v, 'r' );
              plot(    out_save.real_v, 'g');
        legend('aim', 'status','real' );

        figure(2)
        subplot(2,1,1);
        hold off;
        plot(out_save.u);
         subplot(2,1,2);
        hold off;
        plot(out_save.u_ki);
        clear yout error ; 




