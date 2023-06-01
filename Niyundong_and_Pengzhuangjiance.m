clc
clear
close all
%% 障碍物 
cylinderMatrix =[1,-2.5;1,-2;1,-1.5;1,-1;1,-0.5;1,0;1,0.5;1,1;1,1.5;1,2;  -2,-4.5;-2,-4;-2,-3.5;-2,-3;-2,-2.5;-2,-2;-2,-1.5;-2,1.5;-2,2;-2,2.5;-2,3;-2,3.5;-2,4;-2,4.5;];%圆柱体中心坐标（n*2矩阵，没有第三个维度，从z=0开始绘制圆柱体）
cylinderRMatrix =[0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;  0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;0.25;];%圆柱体半径（n*1向量）
cylinderHMatrix =[1.5;1.5;1.5;1.5;1.5;1.5;1.5;1.5;1.5;1.5;5;5;5;5;5;5;5;5;5;5;5;5;5;5;];%圆柱体高（n*1向量）
%% 2.将末端姿态代入机械臂逆运动模型中，求出每个末端点对应的八组逆解角度，将八组逆解角度代入机械臂正运动学方程，判断机械臂与障碍物是否发生碰撞，进行碰撞检测，以实现机械臂避障路径规划。
 
% T(:,1)=x;% 
% T(:,2)=path_(:,1);
% T(:,3)=path_(:,2);%路径上各点坐标（），路径点为10个，有蚁群算法得来
T=[2,-1,0.500000000000000;0.656233002857464,0.243163343387402,3.04918206615122;-2.46361755102331,-1.23278175255090,1.13114300365672;-3,-2,0.500000000000000];
G = [];  %用来存储新的障碍节点，用于栅格地图的更新
%N = ndims(T);%原来的
N=length(T);%修改了应该是 蚂蚁的走过点的数量
Q = []; %可行路径上的最终各点的关节角
c = [0 0 0 0 0 0 0 0]; %不可行点判别数组  % A为路径可行判别数，可行为1，不可行为0
sum = [];
for i =1:N    %分别对各点进行碰撞检测
    t = T(i,:);
    q = transl(t);%换算成可以计算的形式
        TH = niyundx_change(q);
  
    for m = 1:8  %根据转角最小原则进行排序，小的放前面，大的放后面
        sum(m) = abs(TH(m,1))+abs(TH(m,2))+abs(TH(m,3))+abs(TH(m,4))+abs(TH(m,5))+abs(TH(m,6));
    end
    [sim,index] = sort(sum);
    THH = [TH(index(1),:);TH(index(2),:);TH(index(3),:);TH(index(4),:);TH(index(5),:);
        TH(index(6),:);TH(index(7),:);TH(index(8),:)];
    
    
    for j = 1:8   %八组关节角分别进行碰撞检测，若有可行的关节角，则后面的不继续检查
         B = pengzhuangjiance(THH(j,:));
         if B == 0
                Q(i,:) = THH(j,:);
                break
         else
             c(j) = 1; %把不可行的关节角相应的判别数组里的位置置1，便于最后的关节可行检测
         end
    end
    cc = 1;
    for n = 1:8
        cc = cc*c(n);
    end
    if cc ==1  %判别i坐标处是否所有关节角都不可行
        G = [G;T(i,:)];
        A = 0  ;%若都不可行，表示路径不可行，该点纳入障碍节点，需重新规划
        break
    else if i == N  %若可行，判断是否已经最后一个路径点
            A = 1;   %若是，则路径可行，输出各点的最优关节角
            Q;
            break
        end
    end
end
%插入轨迹点矩阵，并用三次多项式插值法使轨迹平滑
%Вставьте матрицу траектории и использут  методаТри -E -Terms Interpolation' для сглаживания траектории

ty=Q(:,1);%每一行的第一个为末端的，这一步是为了插值图
ty=ty';
tx=1:N;
tx1=1:0.02:N;
ty1=interp1(tx,ty,tx1,'cubic');%（差值）

%反向
Q2=Q;
Q2_temp=Q2;
for i =1:N
  Q2(i,:)=Q2_temp(N+1-i,:);
end

figure  
plot(tx,ty,'o',tx1,ty1,'r');  %o是原来的点，红的是插入值后
title('Кубическая сплайн-интерполяция')  


%% 关节角带入机器人
q_c=[];%正向
q_c1=[];%返向
for i=1:6
    ty=Q(:,i);
    ty=ty';
    tx=1:N;
    tx1=1:0.02:N;
    q_c(i,:)=interp1(tx,ty,tx1,'cubic');
end
for i=1:6
    tyy1=Q2(:,i);
    tyy1=tyy1';
    txx=1:N;
    txx1=1:0.02:N;
    q_c1(i,:)=interp1(txx,tyy1,txx1,'cubic');
end
q_c=q_c';
q_c1=q_c1';

%% 工具箱设置障碍物  【和前面的一样】

figure


[n,~] = size(cylinderMatrix);
for i = 1:n   %绘制圆柱体障碍物
    [x,y,z] = cylinder(cylinderRMatrix(i));
    z(2,:) = cylinderHMatrix(i);
    surfc(x + cylinderMatrix(i,1),y + cylinderMatrix(i,2),...
          z,'FaceColor','interp');
    hold on;
end


xlabel('x');ylabel('y');zlabel('z');
camlight
hold on

axis([-5,5,-5,5,0,5]);
view([0,50,0])%相机视角

%%
%建立机器人模型  
% % theta d a alpha offset
L(1)=Link([0,0.39581,0.44895,-pi/2]);L(1).qlim=[-pi,pi]; 
L(2)=Link([0,0,3,0]); L(2).qlim=[-(2/3)*pi,(1/4)*pi];
L(3)=Link([0,0,3,0]); L(3).qlim=[-(2/3)*pi,(2/3)*pi];
L(4)=Link([0,0,0.876,-pi/2]); L(3).qlim=[-(2/3)*pi,(2/3)*pi];
L(5)=Link([0,0.84,0,pi/2]); L(4).qlim=[-pi,pi];
L(6)=Link([0,0,1,0]); L(6).qlim=[-(2/3)*pi,(2/3)*pi];
robot=SerialLink(L,'name','BMSTU'); %连接连杆，机器人取名BMSTU
robot.base=transl(0,0,0.865)*trotz(0);

 
v=[35 20];%观看视角【方位角，仰角】
w=[-10,10,-10,10,-0,10];%工作空间大小

robot.plot3d(q,'tilesize',0.5,'tile1color',[0 1 0],'workspace',w,'delay',0.0003,'trail',{'r','LineWidth',3},'path','C:\Users\pc\Desktop\FIX\robot_Solidworks_fix\DaoRu_fix','nowrist','view',v)
%doc plot3d 可以查询这个函数的用法,'color',颜色；  'alpha',1/0 透明
%'floorlevel',L，地板z轴默认-1； 'delay',延迟 ,'tilesize',S 地板上方形瓷砖的边长
%'tile1color',[r g b]偶数瓷砖的颜色 '[no]wrist' 启用显示腕部坐标框    扩展名为.STL 每个STL模型被称为'linkN'.stl，其中N是链接编号0到N

light('Position',[1 1 1],'color','w');
 

%% 碰撞检测部分的函数
function TH =niyundx_change(a)%加入角度限制

%目标位置姿态矩阵
nx=a(1,1);ox=a(1,2);ax=a(1,3);px=a(1,4);
ny=a(2,1);oy=a(2,2);ay=a(2,3);py=a(2,4);
nz=a(3,1);oz=a(3,2);az=a(3,3);pz=a(3,4)-0.865;

% 求解关节角1 [2]
	theta1_1 =atan2(0,sqrt(abs((py-ny).^2+(nx-px).^2)))-atan2(py-ny,nx-px);
	theta1_2 =atan2(0,-sqrt(abs((py-ny).^2+(nx-px).^2)))-atan2(py-ny,nx-px);
% 求解关节角5 [2]theta5不应该为0或者pi
theta5_1=atan2(sqrt(abs((ny*cos(theta1_1)-nx*sin(theta1_1)).^2+(oy*cos(theta1_1)-ox*sin(theta1_1)).^2)),ay*cos(theta1_1)-ax*sin(theta1_1));
theta5_2=atan2(-sqrt(abs((ny*cos(theta1_1)-nx*sin(theta1_1)).^2+(oy*cos(theta1_1)-ox*sin(theta1_1)).^2)),ay*cos(theta1_1)-ax*sin(theta1_1));
theta5_3=atan2(sqrt(abs((ny*cos(theta1_2)-nx*sin(theta1_2)).^2+(oy*cos(theta1_2)-ox*sin(theta1_2)).^2)),ay*cos(theta1_2)-ax*sin(theta1_2));
theta5_4=atan2(-sqrt(abs((ny*cos(theta1_2)-nx*sin(theta1_2)).^2+(oy*cos(theta1_2)-ox*sin(theta1_2)).^2)),ay*cos(theta1_2)-ax*sin(theta1_2));
% 求解关节角6,  [1]
    theta6_1 = atan2((oy*cos(theta1_1)-ox*sin(theta1_1))./sin(theta5_1),(nx*sin(theta1_1)-ny*cos(theta1_1))./sin(theta5_1));
    theta6_2 = atan2((oy*cos(theta1_1)-ox*sin(theta1_1))./sin(theta5_2),(nx*sin(theta1_1)-ny*cos(theta1_1))./sin(theta5_2));
    theta6_3 = atan2((oy*cos(theta1_2)-ox*sin(theta1_2))./sin(theta5_3),(nx*sin(theta1_2)-ny*cos(theta1_2))./sin(theta5_3));
    theta6_4 = atan2((oy*cos(theta1_2)-ox*sin(theta1_2))./sin(theta5_4),(nx*sin(theta1_2)-ny*cos(theta1_2))./sin(theta5_4));
% 求解关节角2，3，4 []  theta2有问题！！！！
   
    q234_1 = atan2(-az./sin(theta5_1), (ax*cos(theta1_1)+ay*sin(theta1_1))./sin(theta5_1));
    q234_2 = atan2(-az./sin(theta5_2), (ax*cos(theta1_1)+ay*sin(theta1_1))./sin(theta5_2));
    
    q234_3 = atan2(-az./sin(theta5_3), (ax*cos(theta1_2)+ay*sin(theta1_2))./sin(theta5_3));
    
    q234_4 = atan2(-az./sin(theta5_4), (ax*cos(theta1_2)+ay*sin(theta1_2))./sin(theta5_4));
    
    
    A_1 = -((1/250)*3*10229.^(0.5))*cos(q234_1+atan(70/73))+(px-nx)*cos(theta1_1)+(py-ny)*sin(theta1_1)-0.44895;
    B_1 =-((1/250)*3*10229.^(0.5))*sin(q234_1+atan(70/73))+nz-pz+0.39581;
    
    A_2 = -((1/250)*3*10229.^(0.5))*cos(q234_2+atan(70/73))+(px-nx)*cos(theta1_1)+(py-ny)*sin(theta1_1)-0.44895;
    B_2 =-((1/250)*3*10229.^(0.5))*sin(q234_2+atan(70/73))+nz-pz+0.39581;
    
    A_3 = -((1/250)*3*10229.^(0.5))*cos(q234_3+atan(70/73))+(px-nx)*cos(theta1_2)+(py-ny)*sin(theta1_2)-0.44895;
    B_3 =-((1/250)*3*10229.^(0.5))*sin(q234_3+atan(70/73))+nz-pz+0.39581;
    
    A_4 = -((1/250)*3*10229.^(0.5))*cos(q234_4+atan(70/73))+(px-nx)*cos(theta1_2)+(py-ny)*sin(theta1_2)-0.44895;
    B_4 =-((1/250)*3*10229.^(0.5))*sin(q234_4+atan(70/73))+nz-pz+0.39581;
    a2=3;
    a3=3;

    theta2_1 = atan2(A_1^2+B_1^2+a2^2-a3^2, sqrt(abs(4*a2^2*(A_1^2+B_1^2)-(A_1^2+B_1^2+a2^2-a3^2)^2)))-atan2(A_1, B_1);
    theta2_2 = atan2(A_1^2+B_1^2+a2^2-a3^2, -sqrt(abs(4*a2^2*(A_1^2+B_1^2)-(A_1^2+B_1^2+a2^2-a3^2)^2)))-atan2(A_1, B_1);
    theta2_3 = atan2(A_2^2+B_2^2+a2^2-a3^2, sqrt(abs(4*a2^2*(A_2^2+B_2^2)-(A_2^2+B_2^2+a2^2-a3^2)^2)))-atan2(A_2, B_2);
    theta2_4 = atan2(A_2^2+B_2^2+a2^2-a3^2, -sqrt(abs(4*a2^2*(A_2^2+B_2^2)-(A_2^2+B_2^2+a2^2-a3^2)^2)))-atan2(A_2, B_2);
    theta2_5 = atan2(A_3^2+B_3^2+a2^2-a3^2, sqrt(abs(4*a2^2*(A_3^2+B_3^2)-(A_3^2+B_3^2+a2^2-a3^2)^2)))-atan2(A_3, B_3);
    theta2_6 = atan2(A_3^2+B_3^2+a2^2-a3^2, -sqrt(abs(4*a2^2*(A_3^2+B_3^2)-(A_3^2+B_3^2+a2^2-a3^2)^2)))-atan2(A_3, B_3);
    theta2_7 = atan2(A_4^2+B_4^2+a2^2-a3^2, sqrt(abs(4*a2^2*(A_4^2+B_4^2)-(A_4^2+B_4^2+a2^2-a3^2)^2)))-atan2(A_4, B_4);
    theta2_8 = atan2(A_4^2+B_4^2+a2^2-a3^2, -sqrt(abs(4*a2^2*(A_4^2+B_4^2)-(A_4^2+B_4^2+a2^2-a3^2)^2)))-atan2(A_4, B_4);


   q23_1 = atan2(B_1-a2*sin(theta2_1),A_1-a2*cos(theta2_1));
   q23_2 = atan2(B_1-a2*sin(theta2_2),A_1-a2*cos(theta2_2));
   q23_3 = atan2(B_2-a2*sin(theta2_3),A_2-a2*cos(theta2_3));
   q23_4 = atan2(B_2-a2*sin(theta2_4),A_2-a2*cos(theta2_4));
   q23_5 = atan2(B_3-a2*sin(theta2_5),A_3-a2*cos(theta2_5));
   q23_6 = atan2(B_3-a2*sin(theta2_6),A_3-a2*cos(theta2_6));
   q23_7 = atan2(B_4-a2*sin(theta2_7),A_4-a2*cos(theta2_7));
   q23_8 = atan2(B_4-a2*sin(theta2_8),A_4-a2*cos(theta2_8));
   

    
    theta3_1 = q23_1 - theta2_1;
    theta3_2 = q23_2 - theta2_2;
    theta3_3 = q23_3 - theta2_3;
    theta3_4 = q23_4 - theta2_4;
    theta3_5 = q23_5 - theta2_5;
    theta3_6 = q23_6 - theta2_6;
    theta3_7 = q23_7 - theta2_7;
    theta3_8 = q23_8 - theta2_8;
    
    theta4_1 = q234_1 - q23_1;
    theta4_2 = q234_1 - q23_2;
    theta4_3 = q234_2 - q23_3;
    theta4_4 = q234_2 - q23_4;
    theta4_5 = q234_3 - q23_5;
    theta4_6 = q234_3 - q23_6;
    theta4_7 = q234_4 - q23_7;
    theta4_8 = q234_4 - q23_8;
%    %输出最终的4个解
TH = [  
              theta1_1,theta2_1,theta3_1,theta4_1,theta5_1,theta6_1;
 	  	      theta1_1,theta2_2,theta3_2,theta4_2,theta5_1,theta6_1;
              theta1_1,theta2_3,theta3_3,theta4_3,theta5_2,theta6_2;
              theta1_1,theta2_4,theta3_4,theta4_4,theta5_2,theta6_2;
              
              
              
              
              theta1_2,theta2_5,theta3_5,theta4_5,theta5_3,theta6_3;
              theta1_2,theta2_6,theta3_6,theta4_6,theta5_3,theta6_3;
             
              theta1_2,theta2_7,theta3_7,theta4_7,theta5_4,theta6_4;
              theta1_2,theta2_8,theta3_8,theta4_8,theta5_4,theta6_4;
            
              ];
end
function B = pengzhuangjiance(TH)
%%%运动学正解求各关节坐标
%获取关节角
theta1 = TH(1);theta2 = TH(2);theta3 = TH(3);
theta4 = TH(4);theta5 = TH(5);theta6 = TH(6);
%根据关节参数求取关节坐标
% 1-6
d1 = 0.39581;
d2 = 0;
d3 = 0;
d4 = 1;
d5 = 0.84;
d6 = 0;
%连杆长度 0-5
a1 = 0.44895;
a2 = 1;
a3 = 1;
a4 = 0.876;
a5=0;
a6 = 1;
%连杆扭角 0-5
alp0 = -pi/2;
alp1 = 0;
alp2 = -pi/2;
alp3 = pi/2;
alp4 = -pi/2;
alp5 = 0;

%加入关节角限制
if theta1<-pi || theta1> pi || theta2< -(2/3)*pi || theta2>(1/4)*pi ||theta3<-(2/3)*pi || theta3> (2/3)*pi ||theta4<-(2/3)*pi|| theta4>(2/3)*pi ||theta5<-pi || theta5> pi ||theta6<-(2/3)*pi || theta6> (2/3)*pi
    B=1;
end  








T16 = eye(4);T15 = eye(4);T14 = eye(4);T13 = eye(4);T12 = eye(4);T11 = eye(4);


for i=1:6
    t1 = eval(['theta' num2str(i)]);% eval把字符串当作命令来执行 num2str把数值转换成字符串
    t2 = eval(['alp' num2str(i-1)]);
    t3 = eval(['d' num2str(i)]);
    t4 = eval(['a' num2str(i-1)]);
    
   %机械臂各杆数据带入
    T(:,:,i)=[cos(t1),-sin(t1),0,t4;
    sin(t1)*cos(t2),cos(t1)*cos(t2),-sin(t2),-t3*sin(t2);
    sin(t1)*sin(t2),cos(t1)*sin(t2),cos(t2),t3*cos(t2);
    0,0,0,1];
end

for k=1:6
   T16 = T16*T(:,:,k);
   
end
for k=1:5
    T15 = T15*T(:,:,k);
   
end
for k=1:4
    T14 = T14*T(:,:,k);

end
for k=1:3
    T13 = T13*T(:,:,k);
   
end
for k=1:2
    T12 = T12*T(:,:,k);
 
end
for k=1:1
    T11 = T11*T(:,:,k);
    
end


%求取关节直线向量并确定各连杆坐标范围
x1 = T11(1,4);y1 = T11(2,4);z1 = T11(3,4);H1 = [x1,y1,z1];
maxx1=max(x1,0);minx1=min(x1,0);maxy1=max(y1,0);miny1=min(y1,0);maxz1=max(z1,0);minz1=min(z1,0);

x2 = T12(1,4);y2 = T12(2,4);z2 = T12(3,4);H2 = [x2-x1,y2-y1,z2-z1];
maxx2=max(x1,x2);minx2=min(x1,x2);maxy2=max(y1,y2);miny2=min(y1,y2);maxz2=max(z1,z2);minz2=min(z1,z2);

x3 = T13(1,4);y3 = T13(2,4);z3 = T13(3,4);H3 = [x3-x2,y3-y2,z3-z2];
maxx3=max(x3,x2);minx3=min(x3,x2);maxy3=max(y3,y2);miny3=min(y3,y2);maxz3=max(z3,z2);minz3=min(z3,z2);

x4 = T14(1,4);y4 = T14(2,4);z4 = T14(3,4);H4 = [x4-x3,y4-y3,z4-z3];
maxx4=max(x3,x4);minx4=min(x3,x4);maxy4=max(y3,y4);miny4=min(y3,y4);maxz4=max(z3,z4);minz4=min(z3,z4);

x5 = T15(1,4);y5 = T15(2,4);z5 = T15(3,4);H5 = [x5-x4,y5-y4,z5-z4];
maxx5=max(x5,x4);minx5=min(x5,x4);maxy5=max(y5,y4);miny5=min(y5,y4);maxz5=max(z5,z4);minz5=min(z5,z4);

x6 = T16(1,4);y6 = T16(2,4);z6 = T16(3,4);H6 = [x6-x5,y6-y5,z6-z5];
maxx6=max(x5,x6);minx6=min(x5,x6);maxy6=max(y5,y6);miny6=min(y5,y6);maxz6=max(z5,z6);minz6=min(z5,z6);





%确定连杆最大半径 即【最粗的连杆的半径】要转移到障碍物上
r = 0.3;
%根据障碍物离 基座坐标系Frame0 最近的坐标(长方体的最近的一个角的坐标)和x,y,z方向最大边长确定八个顶点坐标
%（包络长方体）各面与基坐标系三面平行
%圆柱
X1 = 5.75;
Y1 = 2.25;
Z1 = 0;
L = [0.5 5 1.5];
Lx = L(1);Ly = L(2);Lz = L(3);
X2 = X1+Lx;Y2 = Y1;Z2 = Z1;X3 = X1+Lx;Y3 = Y1+Ly;Z3 = Z1;
X4 = X1;Y4 = Y1+Ly;Z4 = Z1;X5 = X1;Y5 = Y1;Z5 = Z1+Lz;
X6 = X1+Lx;Y6 = Y1;Z6 = Z1+Lz;X7 = X1+Lx;Y7 = Y1+Ly;Z7 = Z1+Lz;
X8 = X1;Y8 = Y1+Ly;Z8 = Z1+Lz;


%球
XX1 = 3;
YY1 = 3;
ZZ1 = 3;
LL = [4 4 2];
LLx = LL(1);LLy = LL(2);LLz = LL(3);
XX2 = XX1+LLx;YY2 = YY1;ZZ2 = ZZ1;XX3 = XX1+LLx;YY3 = YY1+LLy;ZZ3 = ZZ1;
XX4 = XX1;YY4 = YY1+LLy;ZZ4 = ZZ1;XX5 = XX1;YY5 = YY1;ZZ5 = ZZ1+LLz;
XX6 = XX1+LLx;YY6 = YY1;ZZ6 = ZZ1+LLz;XX7 = XX1+LLx;YY7 = YY1+LLy;ZZ7 = ZZ1+LLz;
XX8 = XX1;YY8 = YY1+LLy;ZZ8 = ZZ1+LLz;


%膨化后的坐标求解
%定点1沿xyz三个方向膨化后的坐标
kx1 = [X1 Y1 Z1]-[X2 Y2 Z2];
Nx1 = (r/sqrt((X1-X2)^2+(Y1-Y2)^2+(Z1-Z2)^2))*kx1;
ky1 = [X1 Y1 Z1]-[X4 Y4 Z4];
Ny1 = (r/sqrt((X1-X4)^2+(Y1-Y4)^2+(Z1-Z4)^2))*ky1;
kz1 = [X1 Y1 Z1]-[X5 Y5 Z5];
Nz1= (r/sqrt((X1-X5)^2+(Y1-Y5)^2+(Z1-Z5)^2))*kz1;
P1 = [X1 Y1 Z1]+Nx1+Ny1+Nz1;
%定点2沿xyz三个方向膨化后的坐标
kx2 = [X2 Y2 Z2]-[X1 Y1 Z1];
Nx2 = (r/sqrt((X1-X2)^2+(Y1-Y2)^2+(Z1-Z2)^2))*kx2;
ky2 = [X2 Y2 Z2]-[X3 Y3 Z3];
Ny2 = (r/sqrt((X2-X3)^2+(Y2-Y3)^2+(Z2-Z3)^2))*ky2;
kz2 = [X2 Y2 Z2]-[X6 Y6 Z6];
Nz2 = (r/sqrt((X2-X6)^2+(Y2-Y6)^2+(Z2-Z6)^2))*kz2;
P2 = [X2 Y2 Z2]+Nx2+Ny2+Nz2;
%定点3沿xyz三个方向膨化后的坐标
kx3 = [X3 Y3 Z3]-[X4 Y4 Z4];
Nx3 = (r/sqrt((X3-X4)^2+(Y3-Y4)^2+(Z3-Z4)^2))*kx3;
ky3 = [X3 Y3 Z3]-[X2 Y2 Z2];
Ny3 = (r/sqrt((X2-X3)^2+(Y2-Y3)^2+(Z2-Z3)^2))*ky3;
kz3 = [X3 Y3 Z3]-[X7 Y7 Z7];
Nz3 = (r/sqrt((X3-X7)^2+(Y3-Y7)^2+(Z3-Z7)^2))*kz3;
P3 = [X3 Y3 Z3]+Nx3+Ny3+Nz3;
%定点4沿xyz三个方向膨化后的坐标
kx4 = [X4 Y4 Z4]-[X3 Y3 Z3];
Nx4 = (r/sqrt((X3-X4)^2+(Y3-Y4)^2+(Z3-Z4)^2))*kx4;
ky4 = [X4 Y4 Z4]-[X1 Y1 Z1];
Ny4 = (r/sqrt((X4-X1)^2+(Y4-Y1)^2+(Z4-Z1)^2))*ky4;
kz4 = [X4 Y4 Z4]-[X8 Y8 Z8];
Nz4 = (r/sqrt((X4-X8)^2+(Y4-Y8)^2+(Z4-Z8)^2))*kz4;
P4 = [X4 Y4 Z4]+Nx4+Ny4+Nz4;
%定点5沿xyz三个方向膨化后的坐标
kx5 = [X5 Y5 Z5]-[X6 Y6 Z6];
Nx5 = (r/sqrt((X5-X6)^2+(Y5-Y6)^2+(Z5-Z6)^2))*kx5;
ky5 = [X5 Y5 Z5]-[X8 Y8 Z8];
Ny5 = (r/sqrt((X5-X8)^2+(Y5-Y8)^2+(Z5-Z8)^2))*ky5;
kz5 = [X5 Y5 Z5]-[X1 Y1 Z1];
Nz5 = (r/sqrt((X5-X1)^2+(Y5-Y1)^2+(Z5-Z1)^2))*kz5;
P5 = [X5 Y5 Z5]+Nx5+Ny5+Nz5;
%定点6沿xyz三个方向膨化后的坐标
kx6 = [X6 Y6 Z6]-[X5 Y5 Z5];
Nx6 = (r/sqrt((X5-X6)^2+(Y5-Y6)^2+(Z5-Z6)^2))*kx6;
ky6 = [X6 Y6 Z6]-[X7 Y7 Z7];
Ny6 = (r/sqrt((X6-X7)^2+(Y6-Y7)^2+(Z6-Z7)^2))*ky6;
kz6 = [X6 Y6 Z6]-[X2 Y2 Z2];
Nz6 = (r/sqrt((X6-X2)^2+(Y6-Y2)^2+(Z6-Z2)^2))*kz6;
P6 = [X6 Y6 Z6]+Nx6+Ny6+Nz6;
%定点7沿xyz三个方向膨化后的坐标
kx7 = [X7 Y7 Z7]-[X8 Y8 Z8];
Nx7 = (r/sqrt((X7-X8)^2+(Y7-Y8)^2+(Z7-Z8)^2))*kx7;
ky7 = [X7 Y7 Z7]-[X6 Y6 Z6];
Ny7 = (r/sqrt((X6-X7)^2+(Y6-Y7)^2+(Z6-Z7)^2))*ky7;
kz7 = [X7 Y7 Z7]-[X3 Y3 Z3];
Nz7 = (r/sqrt((X7-X3)^2+(Y7-Y3)^2+(Z7-Z3)^2))*kz7;
P7 = [X7 Y7 Z7]+Nx7+Ny7+Nz7;
%定点8沿xyz三个方向膨化后的坐标
kx8 = [X8 Y8 Z8]-[X7 Y7 Z7];
Nx8 = (r/sqrt((X7-X8)^2+(Y7-Y8)^2+(Z7-Z8)^2))*kx8;
ky8 = [X8 Y8 Z8]-[X5 Y5 Z5];
Ny8 = (r/sqrt((X8-X5)^2+(Y8-Y5)^2+(Z8-Z5)^2))*ky8;
kz8 = [X8 Y8 Z8]-[X4 Y4 Z4];
Nz8 = (r/sqrt((X8-X4)^2+(Y8-Y4)^2+(Z8-Z4)^2))*kz8;
P8 = [X8 Y8 Z8]+Nx8+Ny8+Nz8;

%膨化后的坐标求解
%定点1沿xyz三个方向膨化后的坐标
kxx1 = [XX1 YY1 ZZ1]-[XX2 YY2 ZZ2];
Nxx1 = (r/sqrt((XX1-XX2)^2+(YY1-YY2)^2+(ZZ1-ZZ2)^2))*kxx1;
kyy1 = [XX1 YY1 ZZ1]-[XX4 YY4 ZZ4];
Nyy1 = (r/sqrt((XX1-XX4)^2+(YY1-YY4)^2+(ZZ1-ZZ4)^2))*kyy1;
kzz1 = [XX1 YY1 ZZ1]-[XX5 YY5 ZZ5];
Nzz1= (r/sqrt((XX1-XX5)^2+(YY1-YY5)^2+(ZZ1-ZZ5)^2))*kzz1;
PP1 = [XX1 YY1 ZZ1]+Nxx1+Nyy1+Nzz1;
%定点2沿xyz三个方向膨化后的坐标

kxx2 = [XX2 YY2 ZZ2]-[XX1 YY1 ZZ1];
Nxx2 = (r/sqrt((XX1-XX2)^2+(YY1-YY2)^2+(ZZ1-ZZ2)^2))*kxx2;
kyy2 = [XX2 YY2 ZZ2]-[XX3 YY3 ZZ3];
Nyy2 = (r/sqrt((XX2-XX3)^2+(YY2-YY3)^2+(ZZ2-ZZ3)^2))*kyy2;
kzz2 = [XX2 YY2 ZZ2]-[XX6 YY6 ZZ6];
Nzz2 = (r/sqrt((XX2-XX6)^2+(YY2-YY6)^2+(ZZ2-ZZ6)^2))*kzz2;
PP2 = [XX2 YY2 ZZ2]+Nxx2+Nyy2+Nzz2;
%定点3沿xyz三个方向膨化后的坐标
kxx3 = [XX3 YY3 ZZ3]-[XX4 YY4 ZZ4];
Nxx3 = (r/sqrt((XX3-XX4)^2+(YY3-YY4)^2+(ZZ3-ZZ4)^2))*kxx3;
kyy3 = [XX3 YY3 ZZ3]-[XX2 YY2 ZZ2];
Nyy3 = (r/sqrt((XX2-XX3)^2+(YY2-YY3)^2+(ZZ2-ZZ3)^2))*kyy3;
kzz3 = [XX3 YY3 ZZ3]-[XX7 YY7 ZZ7];
Nzz3 = (r/sqrt((XX3-XX7)^2+(YY3-YY7)^2+(ZZ3-ZZ7)^2))*kzz3;
PP3 = [XX3 YY3 ZZ3]+Nxx3+Nyy3+Nzz3;
%定点4沿xyz三个方向膨化后的坐标
kxx4 = [XX4 YY4 ZZ4]-[XX3 YY3 ZZ3];
Nxx4 = (r/sqrt((XX3-XX4)^2+(YY3-YY4)^2+(ZZ3-ZZ4)^2))*kxx4;
kyy4 = [XX4 YY4 ZZ4]-[XX1 YY1 ZZ1];
Nyy4 = (r/sqrt((XX4-XX1)^2+(YY4-YY1)^2+(ZZ4-ZZ1)^2))*kyy4;
kzz4 = [XX4 YY4 ZZ4]-[XX8 YY8 ZZ8];
Nzz4 = (r/sqrt((XX4-XX8)^2+(YY4-YY8)^2+(ZZ4-ZZ8)^2))*kzz4;
PP4 = [XX4 YY4 ZZ4]+Nxx4+Nyy4+Nzz4;
%定点5沿xyz三个方向膨化后的坐标
kxx5 = [XX5 YY5 ZZ5]-[XX6 YY6 ZZ6];
Nxx5 = (r/sqrt((XX5-XX6)^2+(YY5-YY6)^2+(ZZ5-ZZ6)^2))*kxx5;
kyy5 = [XX5 YY5 ZZ5]-[XX8 YY8 ZZ8];
Nyy5 = (r/sqrt((XX5-XX8)^2+(YY5-YY8)^2+(ZZ5-ZZ8)^2))*kyy5;
kzz5 = [XX5 YY5 ZZ5]-[XX1 YY1 ZZ1];
Nzz5 = (r/sqrt((XX5-XX1)^2+(YY5-YY1)^2+(ZZ5-ZZ1)^2))*kzz5;
PP5 = [XX5 YY5 ZZ5]+Nxx5+Nyy5+Nzz5;
%定点6沿xyz三个方向膨化后的坐标
kxx6 = [XX6 YY6 ZZ6]-[XX5 YY5 ZZ5];
Nxx6 = (r/sqrt((XX5-XX6)^2+(YY5-YY6)^2+(ZZ5-ZZ6)^2))*kxx6;
kyy6 = [XX6 YY6 ZZ6]-[XX7 YY7 ZZ7];
Nyy6 = (r/sqrt((XX6-XX7)^2+(YY6-YY7)^2+(ZZ6-ZZ7)^2))*kyy6;
kzz6 = [XX6 YY6 ZZ6]-[XX2 YY2 ZZ2];
Nzz6 = (r/sqrt((XX6-XX2)^2+(YY6-YY2)^2+(ZZ6-ZZ2)^2))*kzz6;
PP6 = [XX6 YY6 ZZ6]+Nxx6+Nyy6+Nzz6;
%定点7沿xyz三个方向膨化后的坐标
kxx7 = [XX7 YY7 ZZ7]-[XX8 YY8 ZZ8];
Nxx7 = (r/sqrt((XX7-XX8)^2+(YY7-YY8)^2+(ZZ7-ZZ8)^2))*kxx7;
kyy7 = [XX7 YY7 ZZ7]-[XX6 YY6 ZZ6];
Nyy7 = (r/sqrt((XX6-XX7)^2+(YY6-YY7)^2+(ZZ6-ZZ7)^2))*kyy7;
kzz7 = [XX7 YY7 ZZ7]-[XX3 YY3 ZZ3];
Nzz7 = (r/sqrt((XX7-XX3)^2+(YY7-YY3)^2+(ZZ7-ZZ3)^2))*kzz7;
PP7 = [XX7 YY7 ZZ7]+Nxx7+Nyy7+Nzz7;
%定点8沿xyz三个方向膨化后的坐标
kxx8 = [XX8 YY8 ZZ8]-[XX7 YY7 ZZ7];
Nxx8 = (r/sqrt((XX7-XX8)^2+(YY7-YY8)^2+(ZZ7-ZZ8)^2))*kxx8;
kyy8 = [XX8 YY8 ZZ8]-[XX5 YY5 ZZ5];
Nyy8 = (r/sqrt((XX8-XX5)^2+(YY8-YY5)^2+(ZZ8-ZZ5)^2))*kyy8;
kzz8 = [XX8 YY8 ZZ8]-[XX4 YY4 ZZ4];
Nzz8 = (r/sqrt((XX8-XX4)^2+(YY8-YY4)^2+(ZZ8-ZZ4)^2))*kzz8;
PP8 = [XX8 YY8 ZZ8]+Nxx8+Nyy8+Nzz8;

%长方体的六个平面法向量求解

%1,2,5,6点确定平面的法向量
k26 = kz2;k21 = kx1; M2 = cross(k26,k21);
%2,3,6,7点确定平面的法向量
k23 = ky2;M3 = cross(k26,k23);
%3,4,7,8点确定平面的法向量
k34 = kx3;k48 = kz8;M4 = cross(k34,k48);
%1,4,5,8点确定平面的法向量
k14 = ky4;M5 = cross(k14,k48);
%5,6,7,8点确定平面的法向量
k56 = kx5;k58 = ky8;M6 = cross(k56,k58);
%1,2,3,4点确定平面的法向量
M1 = cross(k23,k21);

%1,2,5,6点确定平面的法向量
kk26 = kzz2;kk21 = kxx1; MM2 = cross(kk26,kk21);
%2,3,6,7点确定平面的法向量
kk23 = kyy2;MM3 = cross(kk26,kk23);
%3,4,7,8点确定平面的法向量
kk34 = kxx3;kk48 = kzz8;MM4 = cross(kk34,kk48);
%1,4,5,8点确定平面的法向量
kk14 = kyy4;MM5 = cross(kk14,kk48);
%5,6,7,8点确定平面的法向量
kk56 = kxx5;kk58 = kyy8;MM6 = cross(kk56,kk58);
%1,2,3,4点确定平面的法向量
MM1 = cross(kk23,kk21);




%空间长方体的x,y,z坐标范围
MAXX = P2(1);MINX = P1(1);
MAXY = P3(2);MINY = P1(2);
MAXZ = P5(3);MINZ = P1(3);

MAXX = P2(1);MINX = P1(1);
MAXY = P3(2);MINY = P1(2);
MAXZ = P5(3);MINZ = P1(3);

%2
MAXXX = PP2(1);MINXX = PP1(1);
MAXYY = PP3(2);MINYY = PP1(2);
MAXZZ = PP5(3);MINZZ = PP1(3);

MAXXX = PP2(1);MINXX = PP1(1);
MAXYY = PP3(2);MINYY = PP1(2);
MAXZZ = PP5(3);MINZZ = PP1(3);
%判断直线向量是否与法向量垂直，若垂直不存在交点，若不垂直判断交点是否在平面上
H = [H1;H2;H3;H4;H5;H6];%机械臂关节直线向量（指机械臂连杆）
M = [M1;M2;M3;M4;M5;M6];%障碍物长方体的平面的法向量
MM = [MM1;MM2;MM3;MM4;MM5;MM6];%障碍物长方体的平面的法向量
%把每个连杆和每个面判断
E = ones(6);
EE = ones(6);
for i = 1:6
    for j = 1:6
        if dot(H(i,:),M(j,:))==0 %相乘，垂直即正交则为0 ==不存在交点
            E(i,j) = 1;
        else
             P = eval(['P',num2str(j)]);
             t = ((P(1)-eval(['x',num2str(i)]))*M(j,1)+(P(2)-eval(['y',num2str(i)]))*M(j,2)+(P(3)-eval(['z',num2str(i)]))*M(j,3))/dot(H(i,:),M(j,:));
             x = eval(['x',num2str(i)])+H(i,1)*t;
             y = eval(['y',num2str(i)])+H(i,2)*t;
             z = eval(['z',num2str(i)])+H(i,3)*t;
             if eval(['minx',num2str(i)])<=x&&x<=eval(['maxx',num2str(i)]) && MINX<=x&&x<=MAXX && eval(['miny',num2str(i)])<=y&&y<=eval(['maxy',num2str(i)]) && MINY<=y&&y<=MAXY && eval(['minz',num2str(i)])<=z&&z<=eval(['maxz',num2str(i)]) && MINZ<=z&&z<=MAXZ
                 E(i,j) = 0; %不垂直+交点在平面上==有碰触
                 break;
             else
                 E(i,j) = 1;
             end
        end
        
         if dot(H(i,:),MM(j,:))==0 %相乘，垂直即正交则为0 ==不存在交点
            EE(i,j) = 1;
        else
             PP = eval(['PP',num2str(j)]);
             tt = ((PP(1)-eval(['x',num2str(i)]))*MM(j,1)+(PP(2)-eval(['y',num2str(i)]))*MM(j,2)+(PP(3)-eval(['z',num2str(i)]))*MM(j,3))/dot(H(i,:),MM(j,:));
             xx = eval(['x',num2str(i)])+H(i,1)*tt;
             yy = eval(['y',num2str(i)])+H(i,2)*tt;
             zz = eval(['z',num2str(i)])+H(i,3)*tt;
             if eval(['minx',num2str(i)])<=xx&&xx<=eval(['maxx',num2str(i)]) && MINXX<=xx&&xx<=MAXXX && eval(['miny',num2str(i)])<=yy&&yy<=eval(['maxy',num2str(i)]) && MINYY<=yy&&yy<=MAXYY && eval(['minz',num2str(i)])<=zz&&zz<=eval(['maxz',num2str(i)]) && MINZZ<=zz&&zz<=MAXZZ
                 EE(i,j) = 0; %不垂直+交点在平面上==有碰触
                 break;
             else
                 EE(i,j) = 1;
             end
        end
        
        
        
    end
end
a = 1;
for i = 1:6
    for j = 1:6
        a = a*E(i,j); %只有一个C=0即碰撞则a=0
    end
end












if a*aa==1
    B = 0;%无碰撞
    
else 
    B = 1;%发生碰撞
    
end
end
%将障碍物进行规则化处理，以长方体或者圆柱体将障碍物包括进去，然后机械臂本身具有一定体积，
% 可以将机械臂的连杆以圆柱体进行包络，为了方便起见，则将机械臂等效成线，而对障碍物进行膨胀处理，
% 则机械臂的碰撞检测即可简化为当机械臂的线碰到障碍物的面时可认为发生了碰撞。