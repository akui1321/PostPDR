%读取数据
clc;clear;
filename = 'D:\PostPDR\0435.txt';  
  
% 初始化一个空的cell数组来存储每个历元的结构体  
epochs = {};  
l=0.0;
x=0;
y=0;
% 打开文件并读取数据  
fileID = fopen(filename, 'r');  
if fileID == -1  
    error('无法打开文件 %s', filename);  
end  
 index=0;
 timedel=0;
 accnorm=[];
 steptime=[];
 Pos=[ ];
% 读取文件的每一行  
while ~feof(fileID)  
    % 读取一行数据  
    line = fgetl(fileID);  
      
    % 解析每行数据  
    data = strsplit(line);  
    %毫秒转换为秒
    time=(str2double(data{1}))/1000.0;
    % 提取三轴加速度计数据  
    acc = [str2double(data{2}), str2double(data{3}), str2double(data{4})];  
      
    % 提取三轴陀螺仪数据  
    gyro = [str2double(data{5}), str2double(data{6}), str2double(data{7})];  
      
    % 提取三轴磁力计数据  
    mag = [str2double(data{8}), str2double(data{9}), str2double(data{10})];  
      
    % 创建一个结构体来存储历元数据  
    epoch = struct('time', time, 'acc', acc, 'gyro', gyro, 'mag', mag);  
    epoch.accnorm=sqrt(acc(1)^2+acc(2)^2+acc(3)^2) ;
  % epoch.accnorm=abs(acc(3));
    epoch.q=NaN;
    % 将结构体添加到数组中  
    epochs{end+1} = epoch;  
end  
% 关闭文件  
fclose(fileID);  
  
%航向角初始化
initial_heading = NaN; % 初始化为 NaN，表示尚未初始化  
init_data = []; % 用于存储第一秒内的数据点  
  
% 遍历数据，找到第一秒内的数据点  
sum.acc=0;
sum.gyro=0;
sum.mag=0;
for i = 1:length(epochs)  
    % 获取当前历元的时间  
    current_time = epochs{i}.time;  
      
    % 检查当前历元是否在第一秒内  
    if current_time >= epochs{1}.time && current_time <= epochs{1}.time + 3.0
        % 如果是，将传感器数据添加到第一秒数据数组中  
        %init_data(end+1,:) = [i,epochs{i}.gyro, epochs{i}.mag]; 
        sum.acc=sum.acc+epochs{i}.acc;
        sum.gyro=sum.gyro+epochs{i}.gyro;
        sum.mag= sum.mag+ epochs{i}.mag;
        sum.size=i;
    end  
end  
%计算第一秒陀螺仪、磁力计数据平均值
mean.acc=sum.acc/sum.size;
mean.gyro=sum.gyro/sum.size;
mean.mag=sum.mag/sum.size;
% 调用函数来估计初始姿态角
[init_head,init_theta,init_r] = estInitHead(mean); 
%姿态角转四元数
%init_head=-pi;
epochs{sum.size}.q=att2q(init_head,init_theta,init_r);
%epochs{sum.size}.q=[1,0,0,0];
% 显示初始航向角  
disp(['Initial heading: ', num2str(rad2deg(init_head)), ' degrees']);  
epochs{sum.size}.head=init_head;
eInt=0;
foot=0;
%航迹推算
for i=sum.size+1:length(epochs)  
    %计算水平姿态角
    r=atan2(-epochs{i}.acc(1),-epochs{i}.acc(3));
    theta=atan2(epochs{i}.acc(1),sqrt(epochs{i}.acc(2)*epochs{i}.acc(2)+epochs{i}.acc(3)*epochs{i}.acc(3)));
    %计算航向角

  %六轴补偿
 delt=epochs{i}.time-epochs{i-1}.time;
 %n2b=Cnb(epochs{i-1}.q);
 %坐标轴转换
gyro_yxz(1)=epochs{i}.gyro(1);
gyro_yxz(2)=epochs{i}.gyro(2);
gyro_yxz(3)=epochs{i}.gyro(3);

acc_yxz(1)=epochs{i}.acc(1)/sqrt(epochs{i}.acc(1)^2+epochs{i}.acc(2)^2+epochs{i}.acc(3)^2);
acc_yxz(2)=epochs{i}.acc(2)/sqrt(epochs{i}.acc(1)^2+epochs{i}.acc(2)^2+epochs{i}.acc(3)^2);
acc_yxz(3)=epochs{i}.acc(3)/sqrt(epochs{i}.acc(1)^2+epochs{i}.acc(2)^2+epochs{i}.acc(3)^2);
[epochs{i}.gyro,eInt]=sixFilt(acc_yxz,gyro_yxz,epochs{i-1}.q,eInt,delt,epochs{i-1}.accnorm,epochs{i}.accnorm);
%
   %九轴补偿
 gyro_yxz(1)=epochs{i}.gyro(1);
 gyro_yxz(2)=epochs{i}.gyro(2);
 gyro_yxz(3)=epochs{i}.gyro(3);
 
 acc_yxz(1)=epochs{i}.acc(1)/sqrt(epochs{i}.acc(1)^2+epochs{i}.acc(2)^2+epochs{i}.acc(3)^2);
 acc_yxz(2)=epochs{i}.acc(2)/sqrt(epochs{i}.acc(1)^2+epochs{i}.acc(2)^2+epochs{i}.acc(3)^2);
 acc_yxz(3)=epochs{i}.acc(3)/sqrt(epochs{i}.acc(1)^2+epochs{i}.acc(2)^2+epochs{i}.acc(3)^2);
 
 mag_yxz(1)=epochs{i}.mag(1)/sqrt(epochs{i}.mag(1)^2+epochs{i}.mag(2)^2+epochs{i}.mag(3)^2);
 mag_yxz(2)=epochs{i}.mag(2)/sqrt(epochs{i}.mag(1)^2+epochs{i}.mag(2)^2+epochs{i}.mag(3)^2);
 mag_yxz(3)=epochs{i}.mag(3)/sqrt(epochs{i}.mag(1)^2+epochs{i}.mag(2)^2+epochs{i}.mag(3)^2);
%[epochs{i}.gyro,eInt]=nineFilt(acc_yxz,gyro_yxz,mag_yxz,eInt,delt,epochs{i-1}.q);
   %补偿后再进行航向角计算

    %利用陀螺仪数据计算航向角
   %[epochs{i}.head]=gyrohead(r,theta,epochs{i}.gyro(3),epochs{i-1}.head);
    %利用磁力计数据计算航向角
    %epochs{i}.head=maghead(r,theta,epochs{i}.mag);
   
%if epochs{i}.head>pi
%    epochs{i}.head=epochs{i}.head-2*pi;
%elseif epochs{i}.head<-pi
%    epochs{i}.head=epochs{i}.head+2*pi;
%end

    

    %积分法获取航向角以及四元数
delt=epochs{i}.time-epochs{i-1}.time;
%[epochs{i}.q,epochs{i}.head]= getQuatW(epochs{i-1}.q,epochs{i}.gyro,delt);
[epochs{i}.q,epochs{i}.head]=  getQuatWfour(epochs{i-1}.q,epochs{i-1}.gyro,epochs{i}.gyro,delt);
if epochs{i}.head>pi
      epochs{i}.head=epochs{i}.head-2*pi;
elseif epochs{i}.head<-pi
     epochs{i}.head=epochs{i}.head+2*pi;
end
% %姿态角转四元数、
  [epochs{i}.q] = att2q(epochs{i}.head,theta,r);
    a(i-sum.size,1)=epochs{i}.time-epochs{sum.size}.time;
    a(i-sum.size,2)=rad2deg(epochs{i}.head);
    a(i-sum.size,3)=epochs{i}.accnorm ;
    a(i-sum.size,4)=epochs{i}.mag(1) ;
    a(i-sum.size,5)=epochs{i}.mag(2) ;
    a(i-sum.size,6)=epochs{i}.mag(3) ;
    %脚步探测
     [index,l,accnorm,steptime] = detectFoot(epochs{i}.time,accnorm,epochs{i}.accnorm,steptime);
     if index==1
         %计算步频
         foot=foot+1;
         if size(Pos,1)<1
             Pos(foot,:)=[0,0,0,0];
         end
         Pos(foot,1)=x+l*cos(epochs{i}.head);
         Pos(foot,2)=y+l*sin(epochs{i}.head);
         Pos(foot,3)=epochs{i}.time-epochs{sum.size}.time;
         Pos(foot,4)=epochs{i}.accnorm;
         x= Pos(foot,1);
         y= Pos(foot,2);
         disp(['Time',num2str(epochs{i}.time),'X:', num2str(Pos(foot,1)), ' Y:',num2str(Pos(foot,2))]); 
     end
end
figure
scatter(Pos(2:51,1) ,Pos(2:51,2),45,'filled','ColorVariable',[92,158,173]/255.0);
hold on
scatter(Pos(1,1),Pos(1,2),45,'filled','ColorVariable',[239,118,108]/255.0);
hold on
scatter(Pos(52,1),Pos(52,2),45,'filled','ColorVariable',[239,118,108]/255.0);
axis equal
xlabel("N(m)")
ylabel("E(m)")
figure
plot(a(:,1),a(:,2));
delx=sqrt((Pos(52,1)-Pos(1,1))^2+(Pos(52,2)-Pos(1,2))^2)/40.0;
%legend("X","Y","Z");
hold on
%scatter(Pos(:,3),Pos(:,4));
%for i=1:size(a(:,3),1)-1
%    b(i,1)=a(i+1,3)-a(i,3);
%end
%figure
%plot(a(1:end-1,1),b(:,1));

