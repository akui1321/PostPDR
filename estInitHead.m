%初始化航向角
function [init_head,theta,r]= estInitHead(epochs)  
    r=atan2(-epochs.acc(1),-epochs.acc(3));
    theta=atan2(epochs.acc(1),sqrt(epochs.acc(2)*epochs.acc(2)+epochs.acc(3)*epochs.acc(3)));
    %计算磁力计的水平输出
    mnx= epochs.mag(1)*cos(theta)+epochs.mag(2)*sin(theta)*sin(r)+epochs.mag(3)*sin(theta)*cos(r);
    mny= epochs.mag(2)*cos(r)-epochs.mag(3)*sin(r);
    %根据磁力计的水平输出计算磁北方向
    psim=-atan2(mny,mnx);
    init_head=psim+deg2rad(-2.54);
     if(init_head>pi)
        init_head=init_head-2*pi;
    elseif init_head<-pi
        init_head=init_head+2*pi;
    end
end  
  