%两轮小车程序仿真
clc;
clear;
tic;

g=[0,0];%目标点
g_theta=pi/4;%目标方向

car=[-2,-2];%小车起始点
car_theta=-pi/4;%小车起始方向

L=0.2;%设小车轮间距为0.2m

Kp=0.3;%小车运动速度参数
Ki=0;
Kd=0.2;

Kpo=2;%小车转动角速度参数
Kio=0;
Kdo=0.1;

tstep=0.1;%时间步长  
t=0;%时间累积
    
qk=[car(1)
    car(2)
    car_theta];%小车的当前位置及朝向
accerrd=0;%位置累计误差
accerrt=0;%目标偏离角累计误差
v=0;%小车速度
omega=0;%小车转动速度
carp=[qk(1),qk(2)];%小车当前位置
delta_dis=norm(carp-g);%距终点距离
delta_theta_offset=atan((g(2)-qk(2))/(g(1)-qk(1)))-qk(3);%目标偏离角
deltadisp=0;%上一点的移动距离
deltatp=0;%上一点的转动角度

i=0;

figure(1);%画起始点
r=0.05;
alpha=0:pi/50:2*pi;
x=qk(1)+r*cos(alpha); 
y=qk(2)+r*sin(alpha); 
plot(x,y,"-");
fill(x,y,"b");
grid on;
hold on;

alpha=0:pi/50:2*pi;%画结束点
x=g(1)+r*cos(alpha); 
y=g(2)+r*sin(alpha); 
plot(x,y,"-");
fill(x,y,"r");
hold on;
x1=xlabel('x方向');
x2=ylabel('y方向');
title('小车运动轨迹');

%第一部分：从起始点运动至目标点
while delta_dis>0.01
    
    t=t+tstep;
    i=i+1;
    vot(i,1)=t;%存入时间点
    ptt(i,1)=t;

    axis([-4 2 -4 2]);%在更换目标点时需改变显示区域以得到较好的显示效果
    axis equal;
        
    carp=[qk(1),qk(2)];%小车当前坐标
    delta_dis=norm(carp-g);%距终点距离
    accerrd=accerrd+delta_dis;
          
    if g(1)>qk(1)%目标偏离角需根据目标位置和与小车当前位置所形成角度分情况讨论
        delta_theta_offset=atan((g(2)-qk(2))/(g(1)-qk(1)))-qk(3);
    elseif g(2)>qk(2)
            delta_theta_offset=atan((g(2)-qk(2))/(g(1)-qk(1)))+pi-qk(3);
    else
        delta_theta_offset=atan((g(2)-qk(2))/(g(1)-qk(1)))-pi-qk(3);
    end
    accerrt=accerrt+delta_theta_offset;
    
    while delta_theta_offset>pi || delta_theta_offset<=-pi %对目标偏移角进行处理，使其落在(-pi,pi]
    if delta_theta_offset>pi
        delta_theta_offset=delta_theta_offset-2*pi;
    end
    if delta_theta_offset<=-pi
        delta_theta_offset=delta_theta_offset+2*pi;
    end
    end
    
    v=Kp*delta_dis+Ki*tstep*accerrd+Kd*(delta_dis-deltadisp)/tstep;%PID调整速度
    
    omega=Kpo*delta_theta_offset+Kio*tstep*accerrt+Kdo*(delta_theta_offset-deltatp)/tstep;%PID调整角度
    
    Vr=(2*v+L*omega)/2;%通过运动方程解出左轮和右轮的线速度
    Vl=(2*v-L*omega)/2;
    
    while Vr>0.5 || Vl>0.5 || Vr<-0.5 || Vl<-0.5 %Vr Vl限幅
       Vr=Vr*0.9;
       Vl=Vl*0.9;
    end
        
    v=(Vr+Vl)/2;%反解出小车的速度与角速度
    omega=(Vr-Vl)/L;
    
    vot(i,3)=omega;%存入当前时间点小车运动角速度
    vot(i,2)=v;%存入当前时间点小车运动速度
    
    deltadisp=delta_dis;%保存当前的距目标点距离以及偏移角，为下一次循环使用准备
    deltatp=delta_theta_offset;
    
    qkp=qk;
    qk=qk+[v*tstep*cos(qk(3)) %更新当前位置
           v*tstep*sin(qk(3))
           omega*tstep];
    
    figure(1);
    plot([qkp(1),qk(1)],[qkp(2),qk(2)],"-r",'LineWidth',0.5); %画路径
    hold on;
    plot(qk(1),qk(2),"o",'MarkerEdgeColor','g',...%画每个采样点
                         'MarkerFaceColor','g',...
                         'MarkerSize',2);
    hold on;
    
    ptt(i,2)=qk(3);
    ptt(i,3)=Vl;
    ptt(i,4)=Vr;
    
end

figure(2);%画第一部分小车行进速度及角速度关于时间t的图像
grid on;
x1=xlabel('时间t s');
x2=ylabel('行进速度v m/s 角速度ω rad/s');
title('第一部分小车行进速度及角速度关于时间t的图像');
hold on;

for j=1:i
    plot(vot(j,1),vot(j,2),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    plot(vot(j,1),vot(j,3),"-o",'MarkerEdgeColor','r','MarkerFaceColor','r',"MarkerSize",2);
    hold on;
end

legend('行进速度v','角速度ω');

figure(4);%画小车与x轴所成角度时间t的图像
grid on;
hold on;
for j=1:i
    plot(ptt(j,1),ptt(j,2),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    hold on;
end

figure(5);;%画小车左右轮线速度Vl、Vr关于时间t的图像
grid on;
hold on;
for j=1:i
    plot(ptt(j,1),ptt(j,3),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    plot(ptt(j,1),ptt(j,4),"-o",'MarkerEdgeColor','r','MarkerFaceColor','r',"MarkerSize",2);
    hold on;
end

%第二部分，将小车原地旋转至目标方向
dire_offset=g_theta-qk(3);%计算方向偏离角
tp=t;
n=i;
v=0;%到目标点后设小车的速度为0
t=0;
i=0;

accerrdo=0;%方向偏离角累计误差

while abs(dire_offset)>0.001
    
    t=t+tstep;
    i=i+1;
    vo2t(i,1)=t;%存入时间点
    ptt(i+n,1)=tp+t;
    
    dire_offset=g_theta-qk(3);%计算方向偏离角
        
    while dire_offset>pi || dire_offset<=-pi %对偏移角进行处理，使其落在(-pi,pi]
    if dire_offset>pi
        dire_offset=dire_offset-2*pi;
    end
    if dire_offset<=-pi
        dire_offset=dire_offset+2*pi;
    end
    end
    
    omega=Kpo*dire_offset+Kio*tstep*accerrdo+Kdo*(dire_offset-deltatp)/tstep;
    deltatp=dire_offset;
    
    Vr=(2*v+L*omega)/2;%通过运动方程反解出左轮和右轮的线速度
    Vl=(2*v-L*omega)/2;
    
    while Vr>0.5 || Vl>0.5 || Vr<-0.5 || Vl<-0.5 %Vr Vl限幅
       Vr=Vr*0.9;
       Vl=Vl*0.9;
    end
    
    v=(Vr+Vl)/2;
    omega=(Vr-Vl)/L;
    
    vo2t(i,2)=v;
    vo2t(i,3)=omega;
    
    qkp=qk;
    qk=qk+[v*tstep*cos(qk(3)) %更新当前位置
           v*tstep*sin(qk(3))
           omega*tstep];
       
    ptt(i+n,2)=qk(3);   
    ptt(i+n,3)=Vl;
    ptt(i+n,4)=Vr;
    
end

figure(3);%画小车第二部分角速度关于时间t的图像
grid on;
x1=xlabel('时间t s');
x2=ylabel('行进速度v m/s 角速度ω rad/s');
title('第二部分小车角速度关于时间t的图像');
axis([-0.5 3.5 -0.5 3.5]);%在更换目标点时需改变显示区域以得到较好的显示效果
hold on;
for j=1:i
    plot(vo2t(j,1),vo2t(j,2),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    plot(vo2t(j,1),vo2t(j,3),"-o",'MarkerEdgeColor','r','MarkerFaceColor','r',"MarkerSize",2);
    hold on;
end

legend('行进速度v','角速度ω');

figure(4);%画小车与x轴所成角度关于时间t的图像
grid on;
x1=xlabel('时间t s');
x2=ylabel('方向角θ rad');
title('小车与x轴所成角度关于时间t的图像');
hold on;
for j=n+1:n+i
    plot(ptt(j,1),ptt(j,2),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    hold on;
end

figure(5);%画小车左右轮线速度Vl、Vr关于时间t的图像
grid on;
x1=xlabel('时间t s');
x2=ylabel('速度Vl Vr m/s');
title('小车左右轮线速度Vl、Vr关于时间t的图像');
hold on;
for j=n+1:n+i
    plot(ptt(j,1),ptt(j,3),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    plot(ptt(j,1),ptt(j,4),"-o",'MarkerEdgeColor','r','MarkerFaceColor','r',"MarkerSize",2);
    hold on;
end
axis([-1 ptt(n+i,1)+1 -0.55 0.55]);
legend('左轮线速度Vl','右轮线速度Vr');