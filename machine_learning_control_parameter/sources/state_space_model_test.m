%����Ƕ�-�����ٶ�
close all;
clear all;
ang_ctl_vx_vy1;
LEN = length(Angle_PIT) ;
dts = 0.04;
A = [0.9802  -0.1487   0.08769
     0.1616    0.8087  -0.489 
     0.0576    0.6032   0.5587    ];  
B = [0.002375
     -0.02656
      -0.01374];  
C = [1.432e+04 -1077  363.6];   %�۲�Ƕ�  
D = 0;  
  
Q = [1 0 0 
     0 1 0   
     0 0 10  
   
    ];  
R = 0.1;  
%���������ϵͳ�����Լ����K  
K = lqr(A,B,Q,R);  
Ac = A - B*K  ;  %����Ŀ��ֵ
%��ϵͳ����ģ��  
x0 = [0;0;0]; %��ʼ״̬  
t = 0:1:LEN-1;  
t = t* dts;
%ϵͳ����
% u = zeros(size(t));  
u = Angle_PIT;  
%�ջ�����
% [y,x]=lsim(Ac,B,C,D,u,t,x0);  
%����ϵͳ
sys=ss(A,B,C,D)
% [y,x]=lsim(A,B,C,D,u,t); 
[y,T,xout]=lsim(sys,Angle_PIT,t); 
figure(1)
hold off;
% plot(t,y);  
plot(y);
% hold on 
% plot(u);