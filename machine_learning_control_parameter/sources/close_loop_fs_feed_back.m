% 闭环系统的开环表达
%Expert PID Controller
% clear all;
% 用LQR获得闭环响应
%   1.829e05
%   ------------------------------
%   s^3 + 3165 s^2 + 8653 s + 1e04
% close all;
clear all;
ts=0.04;
 
%系统模型
% sys=tf(5.235e005,[1,87.35,1.047e004,0]);
%速度模型（输入角度，输出加速度）
%  num_s = 145.28 ; 
% den_s = [0,7.9423e-04,2*0.059068*0.028182,1,0];
% 闭环系统
%  num_s =1.829e05 / 1.829; 
% den_s = [0,1,339.6,8917,1e05];
 num_s =1.829e05 / 18.29; 
den_s = [0,1,3165,8653,1e04];
 sys1=tf(num_s,den_s);
 %原始系统RR
 num_s2 = 145.28 ; 
den_s2 = [0,7.9423e-04,2*0.059068*0.028182,1,0];
 sys2=tf(num_s2,den_s2);
%  dsys2=c2d(sys2,ts,'z')
 dsys2=c2d(sys2,ts,'tustin');
 rlocus(dsys2);
[num,den]=tfdata(dsys2,'v');
 %系统前级连系统
 sys_t  = sys1/sys2;
 %pid环
 sys_a = sys_t/(1-sys_t*sys2)
 dsysa=c2d(sys_a,ts,'tustin');
 [numa,dena]=tfdata(dsysa,'v')
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);

%  sys=tf( 1.829e05,[0,1,4.192,1259,0]);

ctrl_u_old= zeros(1,10);
ctrl_out_old= zeros(1,10);

u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0;y_2=0;y_3=0;

x=[0,0,0]';
x2_1=0;
% %  dena=vpa(dena,5)
% %   numa=vpa(numa,5)
% dena=[ 1.0, -3.4754, 3.5052, 1.4278, -6.2689, 5.7413, -0.80531, -3.0543, 2.6041, -0.67447]
%   numa=[ 0.0012808, -0.0053451, 0.0099187, -0.010322, 0.0042051, 0.0047417, -0.0095038, 0.0081175, -0.003983, 0.00088975]
%  
error_1=0;
for k=1:1:1000
time(k)=k*ts;
   
rin(k)=1000;                   %Tracing Jieyue Signal

%系统的输入（角度）
% u(k)=Angle_PIT(k)+0.5; %PID Controller
%  u(k)=100; %PID Controller
u(k)= x(1) ; %PID Controller
%9阶离散表达
%     for ii=2:10
%         ctrl_u_old(10-ii+2) =  ctrl_u_old(10-ii+1);
%     end
%     ctrl_u_old(1) = u(k);
%      for ii=2:10
%         ctrl_out_old(10-ii+2) =  ctrl_out_old(10-ii+1);
%      end
%  
%     new_out =  0
%     for ii=2:10
%         new_out =  new_out - dena(ii)*ctrl_out_old(ii);
%     end
%     for ii=1:10
%         new_out =  new_out + numa(ii)*ctrl_u_old(ii);
%     end
%     ctrl_out_old(1) = new_out ;
%     
    %经过一个离散传递(单步)
    [new_out, ctrl_u_old, ctrl_out_old ] = discreate_function(u(k),numa ,dena, ctrl_u_old,ctrl_out_old);
    
% %三阶离散表达
yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(1)*new_out+num(2)*u_1+num(3)*u_2+num(4)*u_3;
%----------Return of PID parameters------------%
u_3=u_2;u_2=u_1;u_1=new_out;
y_3=y_2;y_2=y_1;y_1=yout(k);

  error(k)=rin(k)-yout(k);  
  x(1)=error(k);                % Calculating P
x2_1=x(2);
x(2)=(error(k)-error_1)/ts;   % Calculating D
x(3)=x(3)+error(k)*ts;        % Calculating I
%角度积分限幅
% x(3) = ABS_LIMIT(x(3),10);
% error_1=error(k);
end
figure(1);
hold off
plot(time,yout,'r');
xlabel('time(s)');ylabel('rin,yout');
 
  
