%四元数积分法获取航向角
function [q,head] = getQuatWfour(Lquat,gyro,gyronew,delt)
        K1(1)=(-gyro(1)*Lquat(2)-gyro(2)*Lquat(3)-gyro(3)*Lquat(4))/2.0;
        K1(2)=(+gyro(1)*Lquat(1)-gyro(2)*Lquat(4)+gyro(3)*Lquat(3))/2.0;
        K1(3)=(+gyro(1)*Lquat(4)+gyro(2)*Lquat(1)-gyro(3)*Lquat(2))/2.0;
        K1(4)=(-gyro(1)*Lquat(3)+gyro(2)*Lquat(2)+gyro(3)*Lquat(1))/2.0;

        K1=K1*delt;

        for i =1:4
           q1(i)= Lquat(i) +K1(i)*0.5;
        end
        %四元数归一化
        norm=sqrt(q1(1)*q1(1)+q1(2)*q1(2)+q1(3)*q1(3)+q1(4)*q1(4));
        for i=1:4
            q1(i)=q1(i)/norm;
        end
        %二阶
        gyro_2=0.5*(gyro+gyronew);
        K2(1)=(-gyro_2(1)*q1(2)-gyro_2(2)*q1(3)-gyro_2(3)*q1(4))/2.0;
        K2(2)=(+gyro_2(1)*q1(1)-gyro_2(2)*q1(4)+gyro_2(3)*q1(3))/2.0;
        K2(3)=(+gyro_2(1)*q1(4)+gyro_2(2)*q1(1)-gyro_2(3)*q1(2))/2.0;
        K2(4)=(-gyro_2(1)*q1(3)+gyro_2(2)*q1(2)+gyro_2(3)*q1(1))/2.0;
        K2=K2*delt;

         q2=Lquat+K2*0.5;
         norm=sqrt(q2(1)*q2(1)+q2(2)*q2(2)+q2(3)*q2(3)+q2(4)*q2(4));
        for i=1:4
            q2(i)=q2(i)/norm;
        end

        K3(1)=(-gyro_2(1)*q2(2)-gyro_2(2)*q2(3)-gyro_2(3)*q2(4))/2.0;
        K3(2)=(+gyro_2(1)*q2(1)-gyro_2(2)*q2(4)+gyro_2(3)*q2(3))/2.0;
        K3(3)=(+gyro_2(1)*q2(4)+gyro_2(2)*q2(1)-gyro_2(3)*q2(2))/2.0;
        K3(4)=(-gyro_2(1)*q2(3)+gyro_2(2)*q2(2)+gyro_2(3)*q2(1))/2.0;
        K3=K3*delt;
        q3=Lquat+K3;
        norm=sqrt(q3(1)*q3(1)+q3(2)*q3(2)+q3(3)*q3(3)+q3(4)*q3(4));
        for i=1:4
            q3(i)=q3(i)/norm;
        end

        K4(1)=(-gyronew(1)*q3(2)-gyronew(2)*q3(3)-gyronew(3)*q3(4));
        K4(2)=(+gyronew(1)*q3(1)-gyronew(2)*q3(4)+gyronew(3)*q3(3));
        K4(3)=(+gyronew(1)*q3(4)+gyronew(2)*q3(1)-gyronew(3)*q3(2));
        K4(4)=(-gyronew(1)*q3(3)+gyronew(2)*q3(2)+gyronew(3)*q3(1));
        K4=0.5*delt*K4;
    
        for i=1:4
            q(i)=Lquat(i)+1.0/6.0*(K1(i)+2*K2(i)+2*K3(i)+K4(i));
        end
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