function [gyronew,eInt] = nineFilt(acc,gyro,mag,eInt,delt,q)
  tau=1.0/delt;
  beta=2.146/tau;
  Kp=2*beta;
  Ki=beta^2;
vx=2*(q(2)*q(4)-q(1)*q(3));
vy=2*(q(1)*q(2)+q(3)*q(4));
vz=q(1)*q(1)-q(2)*q(2)-q(3)*q(3)+q(4)*q(4);

e_acc(1)=acc(2)*vz-acc(3)*vy;
e_acc(2)=acc(3)*vx-acc(1)*vz;
e_acc(3)=acc(1)*vy-acc(2)*vx;

hx=2*mag(1)*(0.5-q(3)*q(3)-q(4)*q(4))+2*mag(2)*(q(2)*q(3)-q(1)*q(4))+2*mag(3)*(q(2)*q(4)+q(1)*q(3));
hy=2*mag(1)*(q(2)*q(3)+q(4)*q(1))+2*mag(2)*(0.5-q(2)*q(2)-q(4)*q(4))+2*mag(3)*(q(3)*q(4)-q(1)*q(2));

bx=sqrt(hx*hx+hy*hy);
bz=2*mag(1)*(q(2)*q(4)-q(1)*q(3))+2*mag(2)*(q(3)*q(4)+q(1)*q(2))+2*mag(3)*(0.5-q(2)*q(2)-q(3)*q(3));

w(1)=2*bx*(0.5-q(3)*q(3)-q(4)*q(4))+2*bz*(q(2)*q(4)-q(1)*q(3));
w(2)=2*bx*(q(2)*q(3)-q(1)*q(4))+2*bz*(q(2)*q(1)+q(4)*q(3));
w(3)=2*bx*(q(1)*q(3)+q(2)*q(4))+2*bz*(0.5-q(2)*q(2)-q(3)*q(3));

e_mag(1)=mag(2)*w(3)-mag(3)*w(2);
e_mag(2)=mag(3)*w(1)-mag(1)*w(3);
e_mag(3)=mag(1)*w(2)-mag(2)*w(1);
eInt=eInt+(e_acc+e_mag)*delt;
gyronew=gyro+(Kp*(e_acc+e_mag)+Ki*eInt);
end