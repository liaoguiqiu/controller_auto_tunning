%Expert PID Controller
  clear all;
  ang_ctl_vx_vy1;
% close all;
ts=0.04;
LEN = length(Angle_PIT) ;
%系统模型
% sys=tf(5.235e005,[1,87.35,1.047e004,0]);
%加速模型（输入角度，输出加速度）
 sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');
 %看精度是否降低
den =  [ 1.0, -0.28199, 0.84563]
num =  [ 0, 116.98, 110.18]
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;
 
error_1=0;
for k=1:1:LEN
time(k)=k*ts;
   
rin(k)=100.0;                   %Tracing Jieyue Signal

%系统的输入（角度）
u(k)=Angle_PIT(k); %PID Controller
 
 
%二阶离散表达
yout(k)=-den(2)*y_1-den(3)*y_2 - 0*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+ 0*u_3;
 

%----------Return of PID parameters------------%
u_3=u_2;u_2=u_1;u_1=u(k);
y_3=y_2;y_2=y_1;y_1=yout(k);
    
%角度积分限幅
% x(3) = ABS_LIMIT(x(3),10);
% error_1=error(k);
end
figure(1);
hold off;
plot(time,axUSER_DATA4,'b',time,yout,'r');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
hold off;
plot(time,axUSER_DATA4-yout,'r');
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

