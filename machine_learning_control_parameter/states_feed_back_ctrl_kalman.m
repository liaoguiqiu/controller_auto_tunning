%利用辨识的系统进行控制器设计，这里采用的状态向量的方式 ,对状态的估计采用kalman滤波器
clear all; 
ts=0.04;
ang_ctl_vx_vy1;  %读取数据文件到matlab workspace
ts=0.04;         % 采样间隔
LEN = length(Angle_PIT) ;  %取得数据长度
test_zero = zeros(1,LEN);   %零向量
%系统传递函数 
num_s = 145.28 ;   %分子系数
den_s = [0,7.9423e-04,2*0.059068*0.028182,1,0];  %分母系数
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
sys=tf(num_s,den_s)             %传递函数 
dsys=c2d(sys,ts,'z')           %离散化为z函数
[num,den]=tfdata(dsys,'v');    
[A,B,C,D]=tf2ss(num_s,den_s) ;%将传递函数转化状态空间模型
sys=ss(A,B,C,D)   
 pdobsv(A,C)   %可观与否
[G,H]=c2d(A,B,ts);%离散状态空间
t = 0:1:300;    %时间序列
%开环系统 
x_close_init = [0;0;0]; %状态初始化
% 迭代方式
this = x_close_init;  %状态初始化
y_this = 0;          %输出初始化

%-------------对辨识系统的验证开始---------------------------%
for ii = 1:1:LEN
    %利用实验数据来验证系统方程
    % 输入:系统 参数矩阵 ， 状态缓存 ， 角度（利用姿态角控制速度） 
    %输出 ： 系统输出  ， 状态缓存
      [y_this this]=dsys_state_update(G,H,C,D,this,Angle_ROL(ii)); 
      yout(ii) = y_this; 
end
%结果输出比较绘制
figure(1);
hold off
plot( vyUSER_DATA2,'b' );  %实际结果
hold on;
plot( yout,'r');     %辨识输出结果
xlabel('time(s)');ylabel('rin,yout'); 
%-------------对辨识系统的验证结束---------------------------%

 

%-----------------方式一：对该系统进行控制器设计(直接认为控制状态就是系统状态)开始--------------------%
%tips : 这里调用了matlab内部系统仿真函数dlsim进行仿真 ，可以适当了解dlsim 内部
%LQR方式求反馈 
Q = [1 0 0   
     0 1 0   
     0 0 100];  
R = 0.0000001; 
 K = dlqr(G,H,Q,R);  %LQR方式求反馈增益向量 
Gc = G - H*K  ;  %change to close sys,参照网址  http://blog.csdn.net/heyijia0327/article/details/39270597 
%或者参照网址  http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace
x_close_init = [0;0;0.007];  %状态初始化
u = zeros(size(t));   %目标输出（目标的yout）        
[yout2,xout]=dlsim(Gc,H,C,D,u,x_close_init);  
%结果绘制
figure(3);
 hold off;
plot( yout2,'r');
xlabel('time(40ms)');ylabel('rin,yout');
%-----------------方式一：对该系统进行控制器设计(直接认为控制状态就是系统状态)结束--------------------%



%-----------------方式2认为状态估计有误差(这里利用kalman滤波) ：利用循环实现 --------------------/
% tips ：这里使用自己编写的系统转移函数 dsys_state_update，方便体现嵌入控制循环的架构
x_close_init = [0;0;0.007]; %状态初始化
this = x_close_init;        % 观测初始化
%kalman init

dim_observe = 1;      %观测值维数
n =3;  %状态维数，观测状态每个维度都有1个速度，故需乘2
filter.A  = G;      % kalman状态转移矩阵
filter.A(1,1)  = filter.A(1,1) * 0.80; %实际模型误差
filter.A(1,2)  = filter.A(1,2) * 0.80; %实际模型误差
%看来这种方式能够承受的误差有限
 pdobsv(filter.A,C)   %可观与否
filter.B  = H;     %  kalman控制矩阵
filter.u = 0 -  K * this;  % kalman输入初始化
filter.P = eye(n);       % kalman  协方差
filter.K = zeros(n);      % kalman滤波增益
filter.H =  C;            %% kalman观测矩阵
cQ = 1e-2;
cR = 1e-5;
filter.Q =  cQ* eye(n);   %这里简单设置Q和R对角线元素都相等，设为不等亦可
filter.R =eye(dim_observe)*cR;
filter.x = x_close_init; %初始状态x0
filter.z= C* x_close_init;  %观测初始化
for ii = 1:1: length(t)
      input = K * ( [0;0;0 ]-   filter.x); %控制量，误差反馈结构 ，目标状态是 [0;0;0 ]
    %tips： 这里 yout = C*xthis ,   C =    1.0e+05 *[0  0  1.829193054908528] ,
       % 所以设定目标状态先设定目标yout ， 再反解目标x = [0  0  a ],如果设定 x = [a  0  0]
       % ,yout就不与之线性对应了
  %tips：当目标状态不是 [0;0;0 ]，如果采用input =[0;0;0 ]-   K *   filter.x 全反馈的结构
    %   要对控制乘Nbar  参照网址    http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace
  
  [y_this this]=dsys_state_update(G,H,C,D,this,input); %系统更新
      filter.z = y_this;   %滤波器观测更新
      filter.u  = input;   % 附有控制量的kalman滤波，需要更新控制量
      filter = Kalman(filter);  %滤波器更新
      
      yout3(ii) = y_this;      %保存输出用于绘图
      xout3(ii,:) = this;      %保存状态用于绘图
      x_est (ii,:) = filter.x; %保存估计状态用于绘图
end

%绘制输出
figure(4);
 hold off;
plot( yout3,'r');
xlabel('time(40ms)');ylabel('rin,yout'); %系统输出 
% 估计状态与实际状态比较
figure(5);
 hold off;
plot( xout3(:,2),'r--'); hold on;
xlabel('time(40ms)');ylabel('xout3(:,2)');%  
plot( x_est(:,2),'b*');
xlabel('time(40ms)');ylabel(' x_est(:,2)');
plot( xout3(:,1),'r--'); hold on;
xlabel('time(40ms)');ylabel('xout3(:,1)');
plot( x_est(:,1),'b*');
xlabel('x_est(:,1)');ylabel('x_est(:,1)');
plot( xout3(:,3),'r--'); hold on;
xlabel('time(40ms)');ylabel(' xout3(:,3)');
plot( x_est(:,3),'b*');
xlabel('time(40ms)');ylabel(' x_est(:,3)');

%-----------------方式2认为状态估计有误差(这里利用kalman滤波) ：利用循环实现结束 --------------------/



%% 绝对值限幅
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
%%
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
%%
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
%%
%离散系统状态转移方程
function [y x]=dsys_state_update(G,H,C,D,x,u)
xdot = G*x + H*u;
x= xdot;
y= C*x ;
end

