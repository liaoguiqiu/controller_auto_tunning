%���ñ�ʶ��ϵͳ���п�������ƣ�������õ�״̬�����ķ�ʽ ,��״̬�Ĺ��Ʋ���kalman�˲���
clear all; 
ts=0.04;
ang_ctl_vx_vy1;  %��ȡ�����ļ���matlab workspace
ts=0.04;         % �������
LEN = length(Angle_PIT) ;  %ȡ�����ݳ���
test_zero = zeros(1,LEN);   %������
%ϵͳ���ݺ��� 
num_s = 145.28 ;   %����ϵ��
den_s = [0,7.9423e-04,2*0.059068*0.028182,1,0];  %��ĸϵ��
%  sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1]);
sys=tf(num_s,den_s)             %���ݺ��� 
dsys=c2d(sys,ts,'z')           %��ɢ��Ϊz����
[num,den]=tfdata(dsys,'v');    
[A,B,C,D]=tf2ss(num_s,den_s) ;%�����ݺ���ת��״̬�ռ�ģ��
sys=ss(A,B,C,D)   
 pdobsv(A,C)   %�ɹ����
[G,H]=c2d(A,B,ts);%��ɢ״̬�ռ�
t = 0:1:300;    %ʱ������
%����ϵͳ 
x_close_init = [0;0;0]; %״̬��ʼ��
% ������ʽ
this = x_close_init;  %״̬��ʼ��
y_this = 0;          %�����ʼ��

%-------------�Ա�ʶϵͳ����֤��ʼ---------------------------%
for ii = 1:1:LEN
    %����ʵ����������֤ϵͳ����
    % ����:ϵͳ �������� �� ״̬���� �� �Ƕȣ�������̬�ǿ����ٶȣ� 
    %��� �� ϵͳ���  �� ״̬����
      [y_this this]=dsys_state_update(G,H,C,D,this,Angle_ROL(ii)); 
      yout(ii) = y_this; 
end
%�������Ƚϻ���
figure(1);
hold off
plot( vyUSER_DATA2,'b' );  %ʵ�ʽ��
hold on;
plot( yout,'r');     %��ʶ������
xlabel('time(s)');ylabel('rin,yout'); 
%-------------�Ա�ʶϵͳ����֤����---------------------------%

 

%-----------------��ʽһ���Ը�ϵͳ���п��������(ֱ����Ϊ����״̬����ϵͳ״̬)��ʼ--------------------%
%tips : ���������matlab�ڲ�ϵͳ���溯��dlsim���з��� �������ʵ��˽�dlsim �ڲ�
%LQR��ʽ���� 
Q = [1 0 0   
     0 1 0   
     0 0 100];  
R = 0.0000001; 
 K = dlqr(G,H,Q,R);  %LQR��ʽ������������ 
Gc = G - H*K  ;  %change to close sys,������ַ  http://blog.csdn.net/heyijia0327/article/details/39270597 
%���߲�����ַ  http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace
x_close_init = [0;0;0.007];  %״̬��ʼ��
u = zeros(size(t));   %Ŀ�������Ŀ���yout��        
[yout2,xout]=dlsim(Gc,H,C,D,u,x_close_init);  
%�������
figure(3);
 hold off;
plot( yout2,'r');
xlabel('time(40ms)');ylabel('rin,yout');
%-----------------��ʽһ���Ը�ϵͳ���п��������(ֱ����Ϊ����״̬����ϵͳ״̬)����--------------------%



%-----------------��ʽ2��Ϊ״̬���������(��������kalman�˲�) ������ѭ��ʵ�� --------------------/
% tips ������ʹ���Լ���д��ϵͳת�ƺ��� dsys_state_update����������Ƕ�����ѭ���ļܹ�
x_close_init = [0;0;0.007]; %״̬��ʼ��
this = x_close_init;        % �۲��ʼ��
%kalman init

dim_observe = 1;      %�۲�ֵά��
n =3;  %״̬ά�����۲�״̬ÿ��ά�ȶ���1���ٶȣ������2
filter.A  = G;      % kalman״̬ת�ƾ���
filter.A(1,1)  = filter.A(1,1) * 0.80; %ʵ��ģ�����
filter.A(1,2)  = filter.A(1,2) * 0.80; %ʵ��ģ�����
%�������ַ�ʽ�ܹ����ܵ��������
 pdobsv(filter.A,C)   %�ɹ����
filter.B  = H;     %  kalman���ƾ���
filter.u = 0 -  K * this;  % kalman�����ʼ��
filter.P = eye(n);       % kalman  Э����
filter.K = zeros(n);      % kalman�˲�����
filter.H =  C;            %% kalman�۲����
cQ = 1e-2;
cR = 1e-5;
filter.Q =  cQ* eye(n);   %���������Q��R�Խ���Ԫ�ض���ȣ���Ϊ�������
filter.R =eye(dim_observe)*cR;
filter.x = x_close_init; %��ʼ״̬x0
filter.z= C* x_close_init;  %�۲��ʼ��
for ii = 1:1: length(t)
      input = K * ( [0;0;0 ]-   filter.x); %�������������ṹ ��Ŀ��״̬�� [0;0;0 ]
    %tips�� ���� yout = C*xthis ,   C =    1.0e+05 *[0  0  1.829193054908528] ,
       % �����趨Ŀ��״̬���趨Ŀ��yout �� �ٷ���Ŀ��x = [0  0  a ],����趨 x = [a  0  0]
       % ,yout�Ͳ���֮���Զ�Ӧ��
  %tips����Ŀ��״̬���� [0;0;0 ]���������input =[0;0;0 ]-   K *   filter.x ȫ�����Ľṹ
    %   Ҫ�Կ��Ƴ�Nbar  ������ַ    http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace
  
  [y_this this]=dsys_state_update(G,H,C,D,this,input); %ϵͳ����
      filter.z = y_this;   %�˲����۲����
      filter.u  = input;   % ���п�������kalman�˲�����Ҫ���¿�����
      filter = Kalman(filter);  %�˲�������
      
      yout3(ii) = y_this;      %����������ڻ�ͼ
      xout3(ii,:) = this;      %����״̬���ڻ�ͼ
      x_est (ii,:) = filter.x; %�������״̬���ڻ�ͼ
end

%�������
figure(4);
 hold off;
plot( yout3,'r');
xlabel('time(40ms)');ylabel('rin,yout'); %ϵͳ��� 
% ����״̬��ʵ��״̬�Ƚ�
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

%-----------------��ʽ2��Ϊ״̬���������(��������kalman�˲�) ������ѭ��ʵ�ֽ��� --------------------/



%% ����ֵ�޷�
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
%%
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
%%
%��ɢϵͳ״̬ת�Ʒ���
function [y x]=dsys_state_update(G,H,C,D,x,u)
xdot = G*x + H*u;
x= xdot;
y= C*x ;
end

