function [uk , pid_struct ] = pid_simple_ctrl(ts,aim,real , pid_struct)
err = aim - real;
pid_struct.olderr= [err,pid_struct.olderr(1:9)];
pid_struct.P = pid_struct.kp  * err;
pid_struct.I = (pid_struct.I + ABS_LIMIT(err,5)*ts );
pid_struct.I =pid_struct.ki*ABS_LIMIT( pid_struct.I ,5);
pid_struct.D = pid_struct.kd*(err-pid_struct.olderr(2))/ts;
pid_struct.out =  pid_struct.P +pid_struct.I +  pid_struct.D;
uk = pid_struct.out;
end