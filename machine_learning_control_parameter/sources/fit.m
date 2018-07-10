%Expert PID Controller
clear all;
close all;
ts=0.04;
%系统模型
% sys=tf(5.235e005,[1,87.35,1.047e004,0]);
%加速模型（输入角度，输出加速度）
 sys=tf(135.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;

kp=0.0002;
ki=0.06;     
kd=0;

error_1=0;
for k=1:1:500
time(k)=k*ts;
   
% rin(k)=100.0;                   %Tracing Jieyue Signal
rin(k)=sin(k/100*pi);   
u(k)=kp*x(1)+kd*x(2)+ki*x(3); %PID Controller

%Expert control rule
% if abs(x(1))>0.8      %Rule1:Unclosed control firstly
%    u(k)=0.45;
% elseif abs(x(1))>0.40        
%    u(k)=0.40;
% elseif abs(x(1))>0.20    
%    u(k)=0.12; 
% elseif abs(x(1))>0.01 
%    u(k)=0.10;   
% end   
% 
% if x(1)*x(2)>0|(x(2)==0)       %Rule2
%    if abs(x(1))>=0.05
%       u(k)=u_1+2*kp*x(1);
%    else
%       u(k)=u_1+0.4*kp*x(1);
%    end
% end
%                                                                                                                                                                                                                                                                                                                                                                                                               
% if (x(1)*x(2)<0&x(2)*x2_1>0)|(x(1)==0)   %Rule3
%     u(k)=u(k);
% end
% 
% if x(1)*x(2)<0&x(2)*x2_1<0   %Rule4
%    if abs(x(1))>=0.05
%       u(k)=u_1+2*kp*error_1;
%    else
%       u(k)=u_1+0.6*kp*error_1;
%    end
% end
% 
% if abs(x(1))<=0.001   %Rule5:Integration separation PI control
%    u(k)=0.5*x(1)+0.010*x(3);
% end

%Restricting the output of controller
if u(k)>=5
   u(k)=5;
end
if u(k)<=-5
   u(k)=-5;
end

%Linear model
% %三阶离散表达
% yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+num(4)*u_3;
%二阶离散表达
yout(k)=-den(2)*y_1-den(3)*y_2 - 0*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+ 0*u_3;
error(k)=rin(k)-yout(k);

%----------Return of PID parameters------------%
u_3=u_2;u_2=u_1;u_1=u(k);
y_3=y_2;y_2=y_1;y_1=yout(k);
   
x(1)=error(k);                % Calculating P
x2_1=x(2);
x(2)=(error(k)-error_1)/ts;   % Calculating D
x(3)=x(3)+error(k)*ts;        % Calculating I
%角度积分限幅
% x(3) = ABS_LIMIT(x(3),10);
% error_1=error(k);
end
figure(1);
plot(time,rin,'b',time,yout,'r');
xlabel('time(s)');ylabel('rin,yout');
figure(2);
plot(time,rin-yout,'r');
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

