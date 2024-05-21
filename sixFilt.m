function [gyronew,eInt] = sixFilt(acc,gyro,q,eInt,delt,lastaccnorm,accnorm)

  tau=1.0/delt;
  beta=2.146/tau;
    Kp=2*beta;
    Ki=beta^2;
  %if(abs(accnorm-lastaccnorm)<0.015*9.8)
  %    Kp=2*beta;
  %    Ki=beta^2;
  %elseif (abs(accnorm-lastaccnorm)<0.05*9.8)
  %     Kp=2*beta*0.5;
  %     Ki=beta^2*1.05;
  %else
  %     Kp=2*beta*0.1;
  %     Ki=beta^2*1.2;
  %end
 
  vx=2*(q(2)*q(4)-q(1)*q(3));
  vy=2*(q(1)*q(2)+q(3)*q(4));
  vz=q(1)*q(1)-q(2)*q(2)-q(3)*q(3)+q(4)*q(4);
 %v=n2b*[0,0,-1]';
% e_acc=cross(acc,v')';
  e_acc(1)=acc(2)*vz-acc(3)*vy;
  e_acc(2)=acc(3)*vx-acc(1)*vz;
  e_acc(3)=acc(1)*vy-acc(2)*vx;

 eInt=eInt+e_acc*delt;
 gyronew=gyro+(Kp*e_acc+Ki*eInt);

% q(1)=q(1)+(-q(2)*gyronew(1)-q(3)*gyronew(2)-q(4)*gyronew(3))*0.5*delt;
% q(2)=pa+(q(1)*gyronew(1)+pb*gyronew(3)-pc*gyronew(2))*0.5*delt;
% q(3)=pb+(q(1)*gyronew(2)-pa*gyronew(3)+pc*gyronew(1))*0.5*delt;
% q(4)=pc+(q(1)*gyronew(3)+pa*gyronew(2)-pb*gyronew(1))*0.5*delt;
%norm=sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
%q(1)=q(1)/norm;
%q(2)=q(2)/norm;
%q(3)=q(3)/norm;
%q(4)=q(4)/norm;
end