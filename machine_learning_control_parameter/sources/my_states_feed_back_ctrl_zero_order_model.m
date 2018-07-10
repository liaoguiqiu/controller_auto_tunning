%Expert PID Controller
%��֤���ģ��׼ȷ�ԣ�����ƿ�����

 clear all;  
 
ts=0.04;
   sys=tf(145.28,[1,0]);
ang_ctl_vx_vy1;
ts=0.04;
LEN = length(Angle_PIT) ;
test_zero = zeros(1,LEN);
%ϵͳ���ݺ��� 
 A= [0 1
     0 0 ];
 B = [0;145.28];
G = [1  ts
     0  0 ];
H  =[0;145.28];
C = [1   0 ];
D = 0;
 
sys = ss(A,B,C,D)
Q = [1 0   
     0 1]; 
     
R = 0.0001; 
K = lqr(A,B,Q,R); 
Nbar = rscale(sys,K)
x=[0,0]';
x2_1=0;
 
error_1=0;
t = 0:1:300;  
 
%����ϵͳ
% [yout,T,xout]=lsim(num,den,Angle_PIT,t);
x_close_init = [0;0];
% [yout,T,xout]=lsim(sys,Angle_ROL,t,x_close_init);
% [yout,T,xout]=lsim(sys,test_zero,t);
% ������ʽ
this = x_close_init;
y_this = 0;
for ii = 1:1:LEN
      [y_this this]=dsys_state_update(G,H,C,D,this,Angle_ROL(ii));
      yout(ii) = y_this; 
end
%  [yout,xout]=dlsim(G,H,C,D,Angle_ROL,x_close_init);

%�������
figure(1);
hold off
 plot( vyUSER_DATA2,'b' );
 hold on;
plot( yout,'r');
xlabel('time(s)');ylabel('rin,yout');
  
% ��ɢϵͳ�ջ�
%���������ϵͳ�����Լ����K  
Q = [1 0   
     0 1]; 
     
R = 0.0001; 
 K = dlqr(G,H,Q,R); 
Gc = G - H*K  ;  %change to close sys
x_close_init = [1000;0];
u =10* ones(size(t));  
[yout2,xout]=dlsim(Gc,H,C,D,u,x_close_init);  
%�������


figure(3);
 hold off;
plot( yout2,'r');
xlabel('time(40ms)');ylabel('rin,yout');

%��ʽ2����ѭ��ʵ��
x_close_init = [1000;0];
this = x_close_init;
Ii = 0 ;
ki = 0.00000
for ii = 1:1: length(t)
%     ״̬���R
    %Ŀ��
  % ״̬�������ƽ�����״̬�ﵽ΢��Ϊ0��״̬��������Ҫ����Ŀ��ֵ�ı���
    aim_pos = 2000;%     ��Ӧ�ó�һ������ϵ��������rscale ��http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace
  aim_v = 10;
    Nbar= 148;
%     input =   aim_pos/148 -  K * this;
input = K*([ aim_pos ; aim_v] - this);
    %������ֻ�Ͽ���
    err = aim_pos - this(1);
     Ii = Ii + ki *err ;
    input  = input + Ii;  %  ���ֽ���
      [y_this this]=dsys_state_update(G,H,C,D,this,input);
      yout3(ii) = y_this; 
      xout3(ii) = this(2);
end

figure(4);
 hold off;
plot( yout3,'r');
xlabel('time(40ms)');ylabel('rin,yout');

figure(5);
 hold off;
plot( xout3,'r');
xlabel('time(40ms)');ylabel('rin,yout');
% ����ֵ�޷�
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
% %%�����ܣ�ϵͳ�ɿ����ж��Լ����״̬������
% %%��������ϵ������A
% %%        �������B
% %%        ���ü���P
% %%��������ɿ����жϽ��
% %%        ״̬��������K
%%-------------------------------------%%
function [str K] = pdctrb(A,B)         %���庯��pdctrb
S = ctrb(A,B);                       %��ɿ����б����S
R = rank(S);                         %��ɿ����б����S����
L = length(A);                        %��ϵ������A��ά��
if R == L                            %�ж�rank(S)�Ƿ����A��ά��
    str = 'ϵͳ��״̬��ȫ�ɿص�!'    %����ɿ����жϽ��
else
    str = 'ϵͳ��״̬����ȫ�ɿص�!'
end
end
%%�����ܣ�ϵͳ�ɹ����ж��Լ�������������
%%��������ϵ������A
% %%        �������B
% %%        ���ü���P
% %%��������ɹ����жϽ��
% %%        �����������H
% %%-------------------------------------%%
function [str H] = pdobsv(A,C)          %���庯��pdobsv
V = obsv(A,C);                         %��ɹ����б����V
R = rank(V);                           %��ɹ����б����V����
L = size(A,1);                          %��ϵ������A��ά��
if R == L                              %�ж�rank(V)�Ƿ����A��ά��
    str = 'ϵͳ��״̬��ȫ�ɹ۵�!'       %����ɹ����жϽ��
else
    str = 'ϵͳ��״̬����ȫ�ɹ۵ģ�'
end
end

%״̬ϵͳ����
function [y x]=dsys_state_update(G,H,C,D,x,u)
xdot = G*x + H*u;
x= xdot;
y= C*x ;
end

