%����С���������
clc;
clear;
tic;

g=[0,0];%Ŀ���
g_theta=pi/4;%Ŀ�귽��

car=[-2,-2];%С����ʼ��
car_theta=-pi/4;%С����ʼ����

L=0.2;%��С���ּ��Ϊ0.2m

Kp=0.3;%С���˶��ٶȲ���
Ki=0;
Kd=0.2;

Kpo=2;%С��ת�����ٶȲ���
Kio=0;
Kdo=0.1;

tstep=0.1;%ʱ�䲽��  
t=0;%ʱ���ۻ�
    
qk=[car(1)
    car(2)
    car_theta];%С���ĵ�ǰλ�ü�����
accerrd=0;%λ���ۼ����
accerrt=0;%Ŀ��ƫ����ۼ����
v=0;%С���ٶ�
omega=0;%С��ת���ٶ�
carp=[qk(1),qk(2)];%С����ǰλ��
delta_dis=norm(carp-g);%���յ����
delta_theta_offset=atan((g(2)-qk(2))/(g(1)-qk(1)))-qk(3);%Ŀ��ƫ���
deltadisp=0;%��һ����ƶ�����
deltatp=0;%��һ���ת���Ƕ�

i=0;

figure(1);%����ʼ��
r=0.05;
alpha=0:pi/50:2*pi;
x=qk(1)+r*cos(alpha); 
y=qk(2)+r*sin(alpha); 
plot(x,y,"-");
fill(x,y,"b");
grid on;
hold on;

alpha=0:pi/50:2*pi;%��������
x=g(1)+r*cos(alpha); 
y=g(2)+r*sin(alpha); 
plot(x,y,"-");
fill(x,y,"r");
hold on;
x1=xlabel('x����');
x2=ylabel('y����');
title('С���˶��켣');

%��һ���֣�����ʼ���˶���Ŀ���
while delta_dis>0.01
    
    t=t+tstep;
    i=i+1;
    vot(i,1)=t;%����ʱ���
    ptt(i,1)=t;

    axis([-4 2 -4 2]);%�ڸ���Ŀ���ʱ��ı���ʾ�����Եõ��Ϻõ���ʾЧ��
    axis equal;
        
    carp=[qk(1),qk(2)];%С����ǰ����
    delta_dis=norm(carp-g);%���յ����
    accerrd=accerrd+delta_dis;
          
    if g(1)>qk(1)%Ŀ��ƫ��������Ŀ��λ�ú���С����ǰλ�����γɽǶȷ��������
        delta_theta_offset=atan((g(2)-qk(2))/(g(1)-qk(1)))-qk(3);
    elseif g(2)>qk(2)
            delta_theta_offset=atan((g(2)-qk(2))/(g(1)-qk(1)))+pi-qk(3);
    else
        delta_theta_offset=atan((g(2)-qk(2))/(g(1)-qk(1)))-pi-qk(3);
    end
    accerrt=accerrt+delta_theta_offset;
    
    while delta_theta_offset>pi || delta_theta_offset<=-pi %��Ŀ��ƫ�ƽǽ��д���ʹ������(-pi,pi]
    if delta_theta_offset>pi
        delta_theta_offset=delta_theta_offset-2*pi;
    end
    if delta_theta_offset<=-pi
        delta_theta_offset=delta_theta_offset+2*pi;
    end
    end
    
    v=Kp*delta_dis+Ki*tstep*accerrd+Kd*(delta_dis-deltadisp)/tstep;%PID�����ٶ�
    
    omega=Kpo*delta_theta_offset+Kio*tstep*accerrt+Kdo*(delta_theta_offset-deltatp)/tstep;%PID�����Ƕ�
    
    Vr=(2*v+L*omega)/2;%ͨ���˶����̽�����ֺ����ֵ����ٶ�
    Vl=(2*v-L*omega)/2;
    
    while Vr>0.5 || Vl>0.5 || Vr<-0.5 || Vl<-0.5 %Vr Vl�޷�
       Vr=Vr*0.9;
       Vl=Vl*0.9;
    end
        
    v=(Vr+Vl)/2;%�����С�����ٶ�����ٶ�
    omega=(Vr-Vl)/L;
    
    vot(i,3)=omega;%���뵱ǰʱ���С���˶����ٶ�
    vot(i,2)=v;%���뵱ǰʱ���С���˶��ٶ�
    
    deltadisp=delta_dis;%���浱ǰ�ľ�Ŀ�������Լ�ƫ�ƽǣ�Ϊ��һ��ѭ��ʹ��׼��
    deltatp=delta_theta_offset;
    
    qkp=qk;
    qk=qk+[v*tstep*cos(qk(3)) %���µ�ǰλ��
           v*tstep*sin(qk(3))
           omega*tstep];
    
    figure(1);
    plot([qkp(1),qk(1)],[qkp(2),qk(2)],"-r",'LineWidth',0.5); %��·��
    hold on;
    plot(qk(1),qk(2),"o",'MarkerEdgeColor','g',...%��ÿ��������
                         'MarkerFaceColor','g',...
                         'MarkerSize',2);
    hold on;
    
    ptt(i,2)=qk(3);
    ptt(i,3)=Vl;
    ptt(i,4)=Vr;
    
end

figure(2);%����һ����С���н��ٶȼ����ٶȹ���ʱ��t��ͼ��
grid on;
x1=xlabel('ʱ��t s');
x2=ylabel('�н��ٶ�v m/s ���ٶȦ� rad/s');
title('��һ����С���н��ٶȼ����ٶȹ���ʱ��t��ͼ��');
hold on;

for j=1:i
    plot(vot(j,1),vot(j,2),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    plot(vot(j,1),vot(j,3),"-o",'MarkerEdgeColor','r','MarkerFaceColor','r',"MarkerSize",2);
    hold on;
end

legend('�н��ٶ�v','���ٶȦ�');

figure(4);%��С����x�����ɽǶ�ʱ��t��ͼ��
grid on;
hold on;
for j=1:i
    plot(ptt(j,1),ptt(j,2),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    hold on;
end

figure(5);;%��С�����������ٶ�Vl��Vr����ʱ��t��ͼ��
grid on;
hold on;
for j=1:i
    plot(ptt(j,1),ptt(j,3),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    plot(ptt(j,1),ptt(j,4),"-o",'MarkerEdgeColor','r','MarkerFaceColor','r',"MarkerSize",2);
    hold on;
end

%�ڶ����֣���С��ԭ����ת��Ŀ�귽��
dire_offset=g_theta-qk(3);%���㷽��ƫ���
tp=t;
n=i;
v=0;%��Ŀ������С�����ٶ�Ϊ0
t=0;
i=0;

accerrdo=0;%����ƫ����ۼ����

while abs(dire_offset)>0.001
    
    t=t+tstep;
    i=i+1;
    vo2t(i,1)=t;%����ʱ���
    ptt(i+n,1)=tp+t;
    
    dire_offset=g_theta-qk(3);%���㷽��ƫ���
        
    while dire_offset>pi || dire_offset<=-pi %��ƫ�ƽǽ��д���ʹ������(-pi,pi]
    if dire_offset>pi
        dire_offset=dire_offset-2*pi;
    end
    if dire_offset<=-pi
        dire_offset=dire_offset+2*pi;
    end
    end
    
    omega=Kpo*dire_offset+Kio*tstep*accerrdo+Kdo*(dire_offset-deltatp)/tstep;
    deltatp=dire_offset;
    
    Vr=(2*v+L*omega)/2;%ͨ���˶����̷�������ֺ����ֵ����ٶ�
    Vl=(2*v-L*omega)/2;
    
    while Vr>0.5 || Vl>0.5 || Vr<-0.5 || Vl<-0.5 %Vr Vl�޷�
       Vr=Vr*0.9;
       Vl=Vl*0.9;
    end
    
    v=(Vr+Vl)/2;
    omega=(Vr-Vl)/L;
    
    vo2t(i,2)=v;
    vo2t(i,3)=omega;
    
    qkp=qk;
    qk=qk+[v*tstep*cos(qk(3)) %���µ�ǰλ��
           v*tstep*sin(qk(3))
           omega*tstep];
       
    ptt(i+n,2)=qk(3);   
    ptt(i+n,3)=Vl;
    ptt(i+n,4)=Vr;
    
end

figure(3);%��С���ڶ����ֽ��ٶȹ���ʱ��t��ͼ��
grid on;
x1=xlabel('ʱ��t s');
x2=ylabel('�н��ٶ�v m/s ���ٶȦ� rad/s');
title('�ڶ�����С�����ٶȹ���ʱ��t��ͼ��');
axis([-0.5 3.5 -0.5 3.5]);%�ڸ���Ŀ���ʱ��ı���ʾ�����Եõ��Ϻõ���ʾЧ��
hold on;
for j=1:i
    plot(vo2t(j,1),vo2t(j,2),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    plot(vo2t(j,1),vo2t(j,3),"-o",'MarkerEdgeColor','r','MarkerFaceColor','r',"MarkerSize",2);
    hold on;
end

legend('�н��ٶ�v','���ٶȦ�');

figure(4);%��С����x�����ɽǶȹ���ʱ��t��ͼ��
grid on;
x1=xlabel('ʱ��t s');
x2=ylabel('����Ǧ� rad');
title('С����x�����ɽǶȹ���ʱ��t��ͼ��');
hold on;
for j=n+1:n+i
    plot(ptt(j,1),ptt(j,2),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    hold on;
end

figure(5);%��С�����������ٶ�Vl��Vr����ʱ��t��ͼ��
grid on;
x1=xlabel('ʱ��t s');
x2=ylabel('�ٶ�Vl Vr m/s');
title('С�����������ٶ�Vl��Vr����ʱ��t��ͼ��');
hold on;
for j=n+1:n+i
    plot(ptt(j,1),ptt(j,3),"-o",'MarkerEdgeColor','b','MarkerFaceColor','b',"MarkerSize",2);
    plot(ptt(j,1),ptt(j,4),"-o",'MarkerEdgeColor','r','MarkerFaceColor','r',"MarkerSize",2);
    hold on;
end
axis([-1 ptt(n+i,1)+1 -0.55 0.55]);
legend('�������ٶ�Vl','�������ٶ�Vr');