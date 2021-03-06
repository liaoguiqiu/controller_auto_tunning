%Expert PID Controller
 clear all;
 
ang_ctl_vx_vy1;
ts=0.04;
LEN = length(Angle_PIT) ;
%系统模型
% sys=tf(5.235e005,[1,87.35,1.047e004,0]);
%加速模型（输入角度，输出加速度）
num_s = 145.28 ; 
den_s = [0,7.9423e-04,2*0.059068*0.028182,1];
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
sys=tf(num_s,den_s)
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

%获得状态空间模型
[A,B,C,D]=tf2ss(num_s,den_s)
sys=ss(A,B,C,D)
%离散
[G,H]=c2d(A,B,ts);
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;
 
error_1=0;
t = 0:1:LEN-1;  
t = t* ts;
%开环系统
% [yout,T,xout]=lsim(num,den,Angle_PIT,t);
x_close_init = [0;0];
[yout,T,xout]=lsim(sys,Angle_ROL,t,x_close_init);
% [yout,T,xout]=lsim(sys,test_zero,t);

%结果绘制
figure(1);
 plot( vyUSER_DATA2,'b' );
 hold on;
plot( yout,'r');
xlabel('time(s)');ylabel('rin,yout');
 

%闭环测试
%由上面这个系统，可以计算出K  
Q = [10 0  
     0 10 ]; 
R = 0.00001; 
K = lqr(A,B,Q,R); 
Ac = A - B*K  ;  %change to close sys
x_close_init = [0;0.01];
t = 0:1:100000;  
t = t* ts;
u = zeros(size(t));  
[yout,xout]=lsim(Ac,B,C,D,u,t,x_close_init);  
%结果绘制
figure(2);
 hold off;
plot( yout,'r');
xlabel('time(s)');ylabel('rin,yout');

% 离散系统闭环
%由上面这个系统，可以计算出K  
Q = [50 0   
     0 1 
    ];  
R = 0.01; 
 K = dlqr(G,H,Q,R); 
Gc = G - H*K  ;  %change to close sys
x_close_init = [0;0.001];
t = 0:1:100;  
t = t* ts;
u = zeros(size(t));  
[yout,xout]=dlsim(Gc,H,C,D,u,x_close_init);  
%结果绘制
figure(3);
 hold off;
plot( yout,'r');
xlabel('time(s)');ylabel('rin,yout');

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

