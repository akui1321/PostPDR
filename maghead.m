%利用磁力计数据计算航向角
function maghead= maghead(r,theta,mag)  
    %计算磁力计的水平输出
    mnx=mag(1)*cos(theta)+mag(2)*sin(theta)*sin(r)+mag(3)*sin(theta)*cos(r);
    mny=mag(2)*cos(r)-mag(3)*sin(r);
    %根据磁力计的水平输出计算磁北方向
    psim=-atan2(mny,mnx);
    maghead=psim+deg2rad(-2.54);
    if(maghead>pi)
        maghead=maghead-2*pi;
    elseif maghead<-pi
        maghead=maghead+2*pi;
    end
end  
  