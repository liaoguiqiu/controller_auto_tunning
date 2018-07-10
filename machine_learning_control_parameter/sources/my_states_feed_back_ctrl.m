%Expert PID Controller
 clear all;  
ts=0.04;
  
ang_ctl_vx_vy1;
ts=0.04;
LEN = length(Angle_PIT) ;
test_zero = zeros(1,LEN);
%系统传递函数 
num_s = 145.28 ; 
den_s = [0,7.9423e-04,2*0.059068*0.028182,1,0];
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
sys=tf(num_s,den_s)
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

[A,B,C,D]=tf2ss(num_s,den_s) ;%获得状态空间模型
sys=ss(A,B,C,D)
 pdobsv(A,C)   %可观与否
 
[G,H]=c2d(A,B,ts);%离散状态空间
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;
 
error_1=0;
t = 0:1:300;  
 
%开环系统
% [yout,T,xout]=lsim(num,den,Angle_PIT,t);
x_close_init = [0;0;0];
% [yout,T,xout]=lsim(sys,Angle_ROL,t,x_close_init);
% [yout,T,xout]=lsim(sys,test_zero,t);
% 迭代方式
this = x_close_init;
y_this = 0;
for ii = 1:1:LEN
      [y_this this]=dsys_state_update(G,H,C,D,this,Angle_ROL(ii));
      yout(ii) = y_this; 
end
%  [yout,xout]=dlsim(G,H,C,D,Angle_ROL,x_close_init);

%结果绘制
figure(1);
hold off
 plot( vyUSER_DATA2,'b' );
 hold on;
plot( yout,'r');
xlabel('time(s)');ylabel('rin,yout');
  
% 离散系统闭环
%由上面这个系统，可以计算出K  
Q = [1 0 0   
     0 1 0   
     0 0 100];  
R = 0.0000001; 
 K = dlqr(G,H,Q,R); 
Gc = G - H*K  ;  %change to close sys
x_close_init = [0;0;0.007];
u = zeros(size(t)); 
%  u = ones(size(t)); 
[yout2,xout]=dlsim(Gc,H,C,D,u,x_close_init);  
%结果绘制


figure(3);
 hold off;
plot( yout2,'r');
xlabel('time(40ms)');ylabel('rin,yout');

%方式2利用循环实现
x_close_init = [0;0;0.007];
this = x_close_init;
for ii = 1:1: length(t)
%     状态误差R
    input =   0 -  K * this;
      [y_this this]=dsys_state_update(G,H,C,D,this,input);
      yout3(ii) = y_this; 
      xout3(ii) = this(3);
end

figure(4);
 hold off;
plot( yout3,'r');
xlabel('time(40ms)');ylabel('rin,yout');

figure(5);
 hold off;
plot( xout3,'r');
xlabel('time(40ms)');ylabel('rin,yout');
% 绝对值限幅
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
function [str K] = pdctrb(A,B)         %定义函数pdctrb
S = ctrb(A,B);                       %求可控性判别矩阵S
R = rank(S);                         %求可控性判别矩阵S的秩
L = length(A);                        %求系数矩阵A的维数
if R == L                            %判断rank(S)是否等于A的维数
    str = '系统是状态完全可控的!'    %输出可控性判断结果
else
    str = '系统是状态不完全可控的!'
end
end
%%程序功能：系统可观性判断以及求解输出反馈阵
%%输入量：系数矩阵A
% %%        输出矩阵B
% %%        配置极点P
% %%输出量：可观性判断结果
% %%        输出反馈矩阵H
% %%-------------------------------------%%
function [str H] = pdobsv(A,C)          %定义函数pdobsv
V = obsv(A,C);                         %求可观性判别矩阵V
R = rank(V);                           %求可观性判别矩阵V的秩
L = size(A,1);                          %求系数矩阵A的维数
if R == L                              %判断rank(V)是否等于A的维数
    str = '系统是状态完全可观的!'       %输出可观性判断结果
else
    str = '系统是状态不完全可观的！'
end
end

%状态系统迭代
function [y x]=dsys_state_update(G,H,C,D,x,u)
xdot = G*x + H*u;
x= xdot;
y= C*x ;
end

