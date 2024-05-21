%四元数积分法获取航向角
function [q,head] = getQuatW(Lquat,gyro,delt)
        del_q(1)=(-gyro(1)*Lquat(2)-gyro(2)*Lquat(3)-gyro(3)*Lquat(4))*delt/2.0;
        del_q(2)=(+gyro(1)*Lquat(1)-gyro(2)*Lquat(4)+gyro(3)*Lquat(3))*delt/2.0;
        del_q(3)=(+gyro(1)*Lquat(4)+gyro(2)*Lquat(1)-gyro(3)*Lquat(2))*delt/2.0;
        del_q(4)=(-gyro(1)*Lquat(3)+gyro(2)*Lquat(2)+gyro(3)*Lquat(1))*delt/2.0;
        for i =1:4
           q(i)= Lquat(i) +del_q(i);
        end
      
        %四元数归一化
        norm=sqrt(q(1)*q(1)+q(2)*q(2)+q(3)*q(3)+q(4)*q(4));
        for i=1:4
            q(i)=q(i)/norm;
        end
        %四元数转方向余弦矩阵
        Cbr(1)=q(1)^2+q(2)^2-q(3)^2-q(4)^2;
        Cbr(2)=2*(q(2)*q(3)-q(1)*q(4));
        Cbr(3)=2*(q(2)*q(4)+q(1)*q(3));
        Cbr(4)=2*(q(2)*q(3)+q(1)*q(4));
        Cbr(5)=q(1)^2-q(2)^2+q(3)^2-q(4)^2;
        Cbr(6)=2*(q(3)*q(4)-q(1)*q(2));
        Cbr(7)=2*(q(2)*q(4)-q(1)*q(3));
        Cbr(8)=2*(q(3)*q(4)+q(1)*q(2));
        Cbr(9)=q(1)^2-q(2)^2-q(3)^2+q(4)^2;
        %计算欧拉角
        Att(1)=atan2(Cbr(8),Cbr(9));
        Att(2)=asin(Cbr(7));
        Att(3)=atan2(Cbr(4),Cbr(1));
        head=Att(3);
end