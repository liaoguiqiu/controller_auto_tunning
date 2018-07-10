%Expert PID Controller
 clear all;
 
ang_ctl_vx_vy1;
ts=0.04;
LEN = length(Angle_PIT) ;
test_zero = zeros(1,LEN);
%ϵͳģ��
% sys=tf(5.235e005,[1,87.35,1.047e004,0]);
%�ٶ�ģ�ͣ�����Ƕȣ�������ٶȣ�
num_s = 145.28 ; 
den_s = [0,7.9423e-04,2*0.059068*0.028182,1,0];
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
sys=tf(num_s,den_s)
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

%���״̬�ռ�ģ��
[A,B,C,D]=tf2ss(num_s,den_s)


sys=ss(A,B,C,D)
sysd=c2d(sys,ts);
Ad = sysd.A;
Bd = sysd.B
Cd = sysd.C
Dd = sysd.D
%��ɢ
[G,H]=c2d(A,B,ts);
% dsys = ss(G,H,C,D);
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;
 
error_1=0;
t = 0:1:LEN-1;  
t = t* ts;
%����ϵͳ
% [yout,T,xout]=lsim(num,den,Angle_PIT,t);
x_close_init = [0;0;0];
% [yout,T,xout]=lsim(sys,Angle_ROL,t,x_close_init);
%��ɢ��ʽ
 [yout,xout]=dlsim(Ad,Bd,Cd,Dd,Angle_ROL,x_close_init);

%�������
figure(1);
hold off;
 plot( vyUSER_DATA2,'b' );
 hold on;
plot( yout,'r');
xlabel('time(s)');ylabel('rin,yout');
 

%�ջ�����
%���������ϵͳ�����Լ����K  
Q = [1000 0 0   
     0 100000 0   
     0 0 100000000];  
R = 0.01; 
K = lqr(A,B,Q,R); 
% [str K]=pdctrb(A,B,[ 0.5*j  -0.5*j  0 ])        %��״̬������<K>��������
Ac = A - B*K  ;  %change to close sys
x_close_init = [0;0;0.01];
t = 0:1:1000;  
t = t* ts;
u = zeros(size(t));  
[yout,xout]=lsim(Ac,B,C,D,u,t,x_close_init); 
%�����ջ�ϵͳ�Ĵ��ݺ���
 [num1,den1]=ss2tf(Ac,B,C,D,1)
  sys=tf(num1,den1)
  dsys  =  c2d(sys,ts,'tustin');
  figure(4);%��ñջ�ϵͳ�ĸ��켣
  rlocus(dsys)
  hold off;
  
%�������
figure(2);
 hold off;
plot( yout,'r');
xlabel('time(s)');ylabel('rin,yout');

% ��ɢϵͳ�ջ�
%���������ϵͳ�����Լ����K  
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
%�������
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
% %%�����ܣ�ϵͳ�ɿ����ж��Լ����״̬������
% %%��������ϵ������A
% %%        �������B
% %%        ���ü���P
% %%��������ɿ����жϽ��
% %%        ״̬��������K
%%-------------------------------------%%
function [str K] = pdctrb(A,B,P)         %���庯��pdctrb
S = ctrb(A,B);                       %��ɿ����б����S
R = rank(S);                         %��ɿ����б����S����
L = length(A);                        %��ϵ������A��ά��
if R == L                            %�ж�rank(S)�Ƿ����A��ά��
    str = 'ϵͳ��״̬��ȫ�ɿص�!';     %����ɿ����жϽ��
    K =acker(A,B,P);                 %��״̬��������K
else
    str = 'ϵͳ��״̬����ȫ�ɿص�!';
end
end
