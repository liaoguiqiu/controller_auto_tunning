%Expert PID Controller
 clear all;
close all;
ang_ctl_vx_vy1;
ts=0.04;
LEN = length(Angle_PIT) ;
%ϵͳģ��
% sys=tf(5.235e005,[1,87.35,1.047e004,0]);
%����ģ�ͣ�����Ƕȣ�������ٶȣ�
num_s = 145.28 ; 
den_s = [0,7.9423e-04,2*0.059068*0.028182,1];
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
sys=tf(num_s,den_s)
dsys=c2d(sys,ts,'z');
[num,den]=tfdata(dsys,'v');

%���״̬�ռ�ģ��
[A,B,C,D]=tf2ss(num_s,den_s)
sys=ss(A,B,C,D)
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;
 
error_1=0;
t = 0:1:LEN-1;  
t = t* ts;
%����ϵͳ
% [yout,T,xout]=lsim(num,den,Angle_PIT,t);
[yout,T,xout]=lsim(sys,Angle_ROL,t);
figure(1);
 plot( ayUSER_DATA5,'b' );
 hold on;
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

