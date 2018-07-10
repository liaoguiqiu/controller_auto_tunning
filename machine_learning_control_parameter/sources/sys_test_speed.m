%Expert PID Controller
  clear all;
% close all;
ang_ctl_vx_vy1;
ts=0.04;
LEN = length(Angle_PIT) ;
%系统模型
% sys=tf(5.235e005,[1,87.35,1.047e004,0]);
%速度模型（输入角度，输出加速度）
 sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1,0]);
%  sys=tf( 1.829e05,[0,1,4.192,1259,0]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

ctrl_u_old= zeros(1,4);
ctrl_out_old= zeros(1,4);

x=[0,0,0]';
x2_1=0;
 
error_1=0;
for k=1:1:LEN
time(k)=k*ts;
   
rin(k)=100.0;                   %Tracing Jieyue Signal

%系统的输入（角度）
u(k)=Angle_PIT(k)+0.5; %PID Controller
 
  [new_out, ctrl_u_old, ctrl_out_old ] = discreate_function(u(k),num ,den, ctrl_u_old,ctrl_out_old);

%----------Return of PID parameters------------%
 
 yout(k) = new_out;
    
%角度积分限幅
% x(3) = ABS_LIMIT(x(3),10);
% error_1=error(k);
end
figure(1);
plot(time,vxUSER_DATA1,'b',time,yout,'r');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
plot(time,(vxUSER_DATA1-yout),'r');
xlabel('time(s)');ylabel('error');

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

