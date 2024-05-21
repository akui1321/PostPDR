function [q] = att2q(yaw_rad,pitch_rad,roll_rad)
%姿态角转四元数
 % 绕X轴旋转（滚转）  
    cy = cos(yaw_rad / 2);  
    sy = sin(yaw_rad / 2);  
      
    % 绕Y轴旋转（俯仰）  
    cp = cos(pitch_rad / 2);  
    sp = sin(pitch_rad / 2);  
      
    % 绕Z轴旋转（偏航）  
    cr = cos(roll_rad / 2);  
    sr = sin(roll_rad / 2);  
      
    % 计算四元数  
    q = [cy * cp * cr + sy * sp * sr;  
         cy * cp * sr - sy * sp * cr;  
         cy * sp * cr + sy * cp * sr;  
         sy * cp * cr - cy * sp * sr];  
    norm=sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
    q(1)=q(1)/norm;
    q(2)=q(2)/norm;
    q(3)=q(3)/norm;
     q(4)=q(4)/norm;
end