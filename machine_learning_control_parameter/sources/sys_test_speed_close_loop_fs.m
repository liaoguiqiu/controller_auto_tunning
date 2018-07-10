% �ջ�ϵͳ�Ŀ������
%Expert PID Controller
% clear all;
% ��LQR��ñջ���Ӧ
%   1.829e05
%   ------------------------------
%   s^3 + 3165 s^2 + 8653 s + 1e04
% close all;
ts=0.1;
 
%ϵͳģ��
% sys=tf(5.235e005,[1,87.35,1.047e004,0]);
%�ٶ�ģ�ͣ�����Ƕȣ�������ٶȣ�
%  num_s = 145.28 ; 
% den_s = [0,7.9423e-04,2*0.059068*0.028182,1,0];
%  num_s =1.829e05 / 1.829; 
% den_s = [0,1,339.6,8917,1e05];
 num_s =1.829e05 / 18.29; 
den_s = [0,1,3165,8653,1e04];
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
sys=tf(num_s,den_s)
%  sys=tf( 1.829e05,[0,1,4.192,1259,0]);
dsys=c2d(sys,ts,'tustin');
[num,den]=tfdata(dsys,'v');
figure(2);
hold off;
rlocus(dsys);

u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;
 
error_1=0;
for k=1:1:1000
time(k)=k*ts;
   
rin(k)=100.0;                   %Tracing Jieyue Signal

%ϵͳ�����루�Ƕȣ�
% u(k)=Angle_PIT(k)+0.5; %PID Controller
 u(k)=1000; %PID Controller
 
 
% %������ɢ���
yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+num(4)*u_3;
%----------Return of PID parameters------------%
u_3=u_2;u_2=u_1;u_1=u(k);
y_3=y_2;y_2=y_1;y_1=yout(k);
    
%�ǶȻ����޷�
% x(3) = ABS_LIMIT(x(3),10);
% error_1=error(k);
end
figure(1);
hold off
plot(time,yout,'r');
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

