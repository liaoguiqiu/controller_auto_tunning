%Expert PID Controller
 clear all;
 
ang_ctl_vx_vy1;
ts=0.04;
LEN = length(Angle_PIT) ;
test_zero = zeros(1,LEN);
%系统模型
% sys=tf(5.235e005,[1,87.35,1.047e004,0]);
%速度模型（输入角度，输出加速度）
num_s = 145.28 ; 
den_s = [0,7.9423e-04,2*0.059068*0.028182,1,0];
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
sys=tf(num_s,den_s)
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

%获得状态空间模型
[A,B,C,D]=tf2ss(num_s,den_s)


sys=ss(A,B,C,D)
sysd=c2d(sys,ts);
Ad = sysd.A;
Bd = sysd.B
Cd = sysd.C
Dd = sysd.D
%离散
[G,H]=c2d(A,B,ts);
% dsys = ss(G,H,C,D);
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;
 
error_1=0;
t = 0:1:LEN-1;  
t = t* ts;
%开环系统
% [yout,T,xout]=lsim(num,den,Angle_PIT,t);
x_close_init = [0;0;0];
% [yout,T,xout]=lsim(sys,Angle_ROL,t,x_close_init);
%离散方式
 [yout,xout]=dlsim(Ad,Bd,Cd,Dd,Angle_ROL,x_close_init);

%结果绘制
figure(1);
hold off;
 plot( vyUSER_DATA2,'b' );
 hold on;
plot( yout,'r');
xlabel('time(s)');ylabel('rin,yout');
 

%闭环测试
%由上面这个系统，可以计算出K  
Q = [1000 0 0   
     0 100000 0   
     0 0 100000000];  
R = 0.01; 
K = lqr(A,B,Q,R); 
% [str K]=pdctrb(A,B,[ 0.5*j  -0.5*j  0 ])        %求状态反馈阵<K>极点配置
Ac = A - B*K  ;  %change to close sys
x_close_init = [0;0;0.01];
t = 0:1:1000;  
t = t* ts;
u = zeros(size(t));  
[yout,xout]=lsim(Ac,B,C,D,u,t,x_close_init); 
%连续闭环系统的传递函数
 [num1,den1]=ss2tf(Ac,B,C,D,1)
  sys=tf(num1,den1)
  dsys  =  c2d(sys,ts,'tustin');
  figure(4);%获得闭环系统的根轨迹
  rlocus(dsys)
  hold off;
  
%结果绘制
figure(2);
 hold off;
plot( yout,'r');
xlabel('time(s)');ylabel('rin,yout');

% 离散系统闭环
%由上面这个系统，可以计算出K  
Q = [1 0 0   
     0 1 0   
     0 0 10];  
R = 0.0000001; 
 K = dlqr(G,H,Q,R); 
Gc = G - H*K  ;  %change to close sys
x_close_init = [0;0;0.007];
t = 0:1:100;  
t = t* ts;
u = zeros(size(t));  
[yout,xout]=dlsim(Gc,H,C,D,u,x_close_init);  
%结果绘制
figure(3);
 hold off;
plot( yout,'r');
xlabel('time(40ms)');ylabel('rin,yout');

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
% %%程序功能：系统可控性判断以及求解状态反馈阵
% %%输入量：系数矩阵A
% %%        输入矩阵B
% %%        配置极点P
% %%输出量：可控性判断结果
% %%        状态反馈矩阵K
%%-------------------------------------%%
function [str K] = pdctrb(A,B,P)         %定义函数pdctrb
S = ctrb(A,B);                       %求可控性判别矩阵S
R = rank(S);                         %求可控性判别矩阵S的秩
L = length(A);                        %求系数矩阵A的维数
if R == L                            %判断rank(S)是否等于A的维数
    str = '系统是状态完全可控的!';     %输出可控性判断结果
    K =acker(A,B,P);                 %求状态反馈矩阵K
else
    str = '系统是状态不完全可控的!';
end
end
