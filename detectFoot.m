function [index,l,accnorm,steptime] = detectFoot(time,accnorm,acc,steptime)
     %峰值检测阈值
     peak_threshold=0.15;
     %脚步探测时间阈值
     time_threshold=1.0;
     H=1.75;
     l=0.0;
     index=0;
     if(size(accnorm(),2)<2)
         accnorm=[accnorm,acc];
         %加速度计数据小于2
         index=0;
     else
         del1=accnorm(2)-acc;
         del2=accnorm(2)-accnorm(1);
         accnorm(1)=accnorm(2);
         accnorm(2)=acc;
        if(del1>peak_threshold&&del2>peak_threshold)
         %if(acc<9.2)
          if(size(steptime(),2)<1)
              steptime=[steptime,time];
              %脚步历元小于3
              index=3;
          elseif (size(steptime(),2)<2&&time-steptime(1)>0.4)
              steptime=[steptime,time];
              index=3;
          elseif size(steptime(),2)==2
              if time-steptime(2)>0.4&&time-steptime(1)>1.0
                  index=1;
                  %方法一：
                 % l=0.7;
                  %方法二：PPT
                  %Sf=1.0/(0.8*(time-steptime(2))+0.2*(steptime(2)-steptime(1)));
                  %l=0.7+0.371*(H-1.6)+0.227*(Sf-1.79)*H/1.6;
                  %方法三：
                  Sf=1.0/(0.8*(time-steptime(2))+0.2*(steptime(2)-steptime(1)));
                  l=0.7+0.371*(H-1.75)+0.227*(Sf-1.79)*H/1.75;

                  steptime(1)=steptime(2);
                  steptime(2)=time;
                  %timedel=time-steptime(1);
              else
                  %脚步出现不稳定
                  l=0.0;
                  index=4;
              end
          end
        else
            %峰值探测失败
              l=0.0;
            index=2;
        end
     end
end