%利用陀螺仪数据计算航向角
function [headnew] = gyrohead(r,theta,gyroz,head)
    headnew=cos(r)*cos(theta)*(gyroz)*0.2+head;
end