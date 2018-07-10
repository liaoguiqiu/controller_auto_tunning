      
        
        figure(1);  % k控制结束后的画图
        subplot(4,1,1)
        hold off;
        plot(out_save.ob_p ); hold on;
        plot(yout,'r' ); hold on;
        plot( rin ,'g');hold on;
        plot( rin+over_shot ,'b');hold on;
        plot( rin+ stable_err,'y');hold on;
        plot(acheive_time,yout(acheive_time),'r*'); hold on;
        plot(stable_time,yout(stable_time),'b*'); hold on;
        legend('observe','yout','aim','overshot','stable','achieve_time','stable_time');
         %--velocity track-%
        subplot(4,1,2)
        hold off;
        plot(  loop.outOuter, 'b' ); hold on;
           plot(   out_save.ob_v, 'r' );
              plot(    out_save.real_v, 'g');
        legend('aim', 'status','real' );
        subplot(4,1,3)
        hold off;
        plot( my_save.bestcost,'b')
         legend('best_cost' );
        subplot(4,1,4)
        hold off;
        plot( my_save.mincost_fater_all_loop ,'r')
        legend('best_cost_AFTER_all_loop' );
        
        
        
     