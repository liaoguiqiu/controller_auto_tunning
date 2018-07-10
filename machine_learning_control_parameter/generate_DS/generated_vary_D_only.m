% ���ڲ������ݼ���ֻ��KP�仯��
clear all; 
ts=0.04;%����ʱ��
%ϵͳģ��
sys=tf(145.28,[0,7.9423e-04,2*0.059068*0.028182,1,0]);%�ٶ�ģ�ͣ�����Ƕȣ�����ٶȣ�
dsys=c2d(sys,ts,'z');                          %����ģ����ɢ
[num,den]=tfdata(dsys,'v');                    %��ȡ���ӷ�ĸ
x=[0,0,0]';                                  %P��I��D�ı���
x2_1=0;

kp_var_step = 0.0001;  %�������ڲ���
kp_max      = 1 ;    %������
kp_min      = 0.01 ; %��С����

ki_var_step = 0.001;  %�������ڲ���
ki_max      = 3 ;    %������
ki_min      = 0 ; %��С����

kd_var_step = 0.001;  %�������ڲ���
kd_max      =  1 ;    %������
kd_min      =0 ;    %��С����

kp=0.03;
ki=1.74189585656528e-09;     
kd= 0.000008321786918629;
u_1=0.0;u_2=0.0;u_3=0.0;          %ϵͳ������ʼ��                 
y_1=0;y_2=0;y_3=0;                %ϵͳ������ʼ��
error_1=0;  
acheive_time = 100 ;

stable_bufferlen = 10;
stable_buffer=zeros(1,stable_bufferlen);

my_save = [];
my_save.acheive_time  =  0; %���ڱ���
my_save.stable_time   =  0;
my_save.overshot    = 0 ;
my_save.P  =  0; %���ڱ���
my_save.I   =  0;
my_save.D    = 0 ;
save_point = 1;  %����asv��ָ��
 for d_i=1:1:((kd_max - kd_min)/kd_var_step)  %kp�仯ѭ��
      kd=kd_min + d_i* kd_var_step;
       u_1=0.0;u_2=0.0;u_3=0.0;          %ϵͳ������ʼ��                 
       y_1=0;y_2=0;y_3=0;                %ϵͳ������ʼ��
       error_1=0; 
       acheive_time = 100 ;    %�״ε���ʱ��
       stable_time  = 100;     %��̬ʱ��
       over_shot = 0 ;
       find_achive =0;
        for k=1:1:100
                time(k)=k*ts;                    %ʱ������     
                rin(k)=100;                      %Ŀ���Ծֵ
                % rin(k)=1000*sin(k/100*pi);   
                % u(k)=ABS_LIMIT(kp*x(1) ,5)+kd*x(2)+ ABS_LIMIT( ki*x(3),5); % ���������
                u(k)= kp*x(1)+kd*x(2)+  ki*x(3); % ���������
                u(k)  = ABS_LIMIT(u(k),10);     %�޷�
                % -----------��ɢϵͳת�Ʒ���-------------------%
                yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(1)*u(k)+num(2)*u_1+num(3)*u_2+num(4)*u_3;
                u_3=u_2;u_2=u_1;u_1=u(k);
                y_3=y_2;y_2=y_1;y_1=yout(k);
                %----------Return of PID parameters------------%
                error(k)=rin(k)-yout(k);       
                x(1)=error(k);                % Calculating P
                x2_1=x(2);;
                x(2)=(error(k)-error_1)/ts;   % Calculating D
                x(3)=x(3)+ABS_LIMIT( error(k),1)*ts;        % Calculating I
                x(3)=ABS_LIMIT( x(3),1);   
                error_1=error(k);
                
             %         �� error_buffer ѭ����λ
                stable_buffer = [ error(k),stable_buffer(1:stable_bufferlen-1)];
                stable_error  = stable_buffer*stable_buffer';
                %��¼��Ӧʱ��
                
                if(find_achive==0&& abs(error(k)) <  ( 0.08*rin(k)))
              %        if(abs(error(k)) <  (0.02*rin(k)))            
                    acheive_time = k ;
                    find_achive = 1 ;
                end
                if(over_shot < (-error(k))) %��¼����
                    over_shot  = - error(k) ;
                end
                %��¼������̬ʱ��
                if(stable_error <  (stable_bufferlen*0.05*rin(k)))
                    stable_time = k ;
                    
%                     ���浽asv
                   my_save.P (save_point) = kp ;
                   my_save.I (save_point) = ki ;
                   my_save.D (save_point) = kd ;
                   my_save.acheive_time(save_point) = acheive_time ;
                   my_save.stable_time(save_point) = stable_time ;
                   my_save.overshot(save_point) = over_shot ;
                   save_point  = save_point +1;
                    break;
                end  
        end
%         figure(1); 
%         hold off;
%         plot(yout,'r' ); hold on;
%         plot(acheive_time,yout(acheive_time),'r*'); hold on;
%         plot( rin +over_shot);hold on;
%         clear yout ; 
 end  % kp�仯ѭ��      
 
save('my_save_vary_kd_only.mat','my_save');

%%����ֵ����
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

