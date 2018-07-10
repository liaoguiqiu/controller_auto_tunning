%Expert PID Controller
 clear all;  
ts=0.04;
  
ang_ctl_vx_vy1;
ts=0.04;
LEN = length(Angle_PIT) ;
test_zero = zeros(1,LEN);
%ϵͳ���ݺ��� 
num_s = 145.28 ; 
den_s = [0,7.9423e-04,2*0.059068*0.028182,1,0];
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
sys=tf(num_s,den_s)
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

[A,B,C,D]=tf2ss(num_s,den_s) ;%���״̬�ռ�ģ��
sys=ss(A,B,C,D)
 pdobsv(A,C)   %�ɹ����
 
[G,H]=c2d(A,B,ts);%��ɢ״̬�ռ�
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;
 
error_1=0;
t = 0:1:300;  
 
%����ϵͳ
% [yout,T,xout]=lsim(num,den,Angle_PIT,t);
x_close_init = [0;0;0];
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
%�������


figure(3);
 hold off;
plot( yout2,'r');
xlabel('time(40ms)');ylabel('rin,yout');

%��ʽ2����ѭ��ʵ��
x_close_init = [0;0;0.007];
this = x_close_init;
for ii = 1:1: length(t)
%     ״̬���R
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

