%% 清空变量
clear
clc
close all
%% 导入数据
[dat_xia] = xlsData();
%% 数据重处理
num_cylinder = size(dat_xia,1);                            %圆柱体障碍物个数
dat_cylinder = cylinder_coor_deal(dat_xia);                %将圆柱体数据存放在元胞数组里，方便访问
%% rrt算法部分
PATH = [];
start =[-4 -2 0.5];
goal =[4 0 1];
bar1 = scatter3(start(1),start(2),start(3),80,"cyan",'filled','o','MarkerEdgeColor','k');hold on
bar2 = scatter3(goal(1),goal(2),goal(3),80,"magenta",'filled',"o",'MarkerEdgeColor','k');
path_best = RRT(num_cylinder,dat_cylinder,start,goal);%每次找到最好路径存放在这个变量里
%注意：这个路径点是经过冗余点处理以后的，所以点很少

%% 画图
hold on   %画在同一个图像上
grid on   % 画网格
for k2=1:size(dat_xia,1)
    plot_cylinder(dat_xia(k2,1:3),dat_xia(k2,4),dat_xia(k2,5),1,rand(1,3));%这个是画圆柱体障碍物的函数
end
axis([-5,5,-5,5,0,5])  %设置图像的可视化范围
axis equal     % 图像坐标轴可视化间隔相等 
hold on;
bar3 = plot3(path_best(:,1),path_best(:,2),path_best(:,3),'-o','Color','r','MarkerSize',4,'MarkerFaceColor','k');
axis([-5,5,-5,5,0,5]);
text(start(1),start(2),start(3),'   Отправная точка'); text(goal(1),goal(2),goal(3),'   Конечная точка');

xlabel('x','fontsize',12);
ylabel('y','fontsize',12);
zlabel('z','fontsize',12);
legend([bar1,bar2,bar3],["Отправная точка","Конечная точка","Траектория движения конечных точек роботизированных рук"])
title('Трехмерное пространство планирования пути','fontsize',12)
 

[n,m,~] = size(path_best);
f(1) = 0;
for i = 2:n
    f(i) = f(i-1) + calculate_distance3(path_best(i-1,:),path_best(i,:));
end
length=f(n)

%% 函数
function path_best = RRT(num_cylinder,dat_cylinder,start,goal)
%% 流程初始化
Delta=0.1;               %设置扩展步长，扩展结点允许的最大距离，这个数据越大迭代越快，但是比最优解效果越差
Pg=0.5;                   %采样概率
max_iter = 100000;          %最大迭代次数，如果超过这个次数还没找到路径则认为找不到路径
Map = [goal(1)-start(1),5,5]; 
%% 构建初始化树
T.x(1) = start(1);
T.y(1) = start(2);
T.z(1) = start(3);
T.xpar(1) = goal(1);
T.ypar(1) = goal(2);
T.zpar(1) = goal(3);
T.dist(1) = 0;
T.indpre(1) = 0;
 
tic
count = 1;
for iter = 1:max_iter
 
    % step1: 在地图上随机采样
    coor_rand = rrt_sample(Pg,Map,goal,start);     %在空间进行随机采样，coor_rand是一个  1×3  的数组,在空间中随机产生一个点xrand ->这个点不能是起点
    % plot3(coor_rand(1),coor_rand(2),coor_rand(3),'r*')%随机选点的图
    
    % step2 : 遍历树，找到最近的父节点
    [coor_near,coor_index] = rrt_near(coor_rand,T);%在已知树的点集合中找到距离这个随机点最近的点xnear
      
    % step3: 扩展得到新的节点
    coor_new = rrt_steer(coor_rand,coor_near,Delta);
 
    % step4: 碰撞检测,发生碰撞就会返回1
    flag2 = collision_checking_cylinder(num_cylinder,dat_cylinder,coor_new,coor_near,Delta,3);%这部分是检测是否和圆柱体障碍物碰撞的参数
 
    if flag2==1%碰撞就停止
        continue;
    end
 
    count = count+1;
    % step5:将新点插入进去
    T.x(count) = coor_new(1);
    T.y(count) = coor_new(2);
    T.z(count) = coor_new(3);
    T.xpar(count) = coor_near(1);
    T.ypar(count) = coor_near(2);
    T.zpar(count) = coor_near(3);
    T.dist(count) = calculate_distance3(coor_new,coor_near);
    T.indpre(count) = coor_index;
    line([coor_near(1),coor_new(1)],[coor_near(2),coor_new(2)],[coor_near(3),coor_new(3)],'LineWidth',1,'color','g');
 axis([-5,5,-5,5,0,5]);
    pause(0.0000000000001); %暂停0.1s，使得RRT扩展过程容易观察；
    % 注意：  pause函数时暂停函数，如果为了显示动画则这部分十分重要，不过不加上则会静止动画，不能展示动图
 
    % step6:每次迭代出新点后都检查一遍是否可以直接和终点相连
    if    collision_checking_cylinder(num_cylinder,dat_cylinder,goal,coor_new,calculate_distance3(goal,coor_new),20)==0
        count = count+1;
        T.x(count) = goal(1);
        T.y(count) = goal(2);
        T.z(count) = goal(3);
        T.xpar(count) = coor_new(1);
        T.ypar(count) = coor_new(2);
        T.zpar(count) = coor_new(3);
        T.dist(count) = calculate_distance3(coor_new,goal);
        T.indpre(count) = 0;
        line([goal(1),coor_new(1)],[goal(2),coor_new(2)],[goal(3),coor_new(3)],'LineWidth',1,'color','g');
       axis([-5,5,-5,5,0,10])
        pause(0.00000001); %暂停0.01s，使得RRT扩展过程容易观察
        break;
    end
 
    
end
 
toc
 
% 算法找到路径点后找到到达终点的父代点集合存储在path变量中
if iter>max_iter
    error('Превышено максимальное количество итераций, планирование пути не удается');
end
path(1,1) = T.x(end);path(1,2) = T.y(end);path(1,3) = T.z(end);
path(2,1) = T.x(end-1);path(2,2) = T.y(end-1);path(2,3) = T.z(end-1);
count2 = 2;
ind_pre = T.indpre(end-1);
if iter<=max_iter
    while ~(ind_pre==0)
        count2 = count2+1;
        path(count2,1) = T.x(ind_pre);
        path(count2,2) = T.y(ind_pre);
        path(count2,3) = T.z(ind_pre);
        ind_pre = T.indpre(ind_pre);
    end
 
end
%% RRT算法找到新点全部集合，接下来要去除冗余点
path_best = delete_redundant_points(path,num_cylinder,dat_cylinder);

% line(path_best(:,1),path_best(:,2),path_best(:,3),'LineWidth',3,'Color','k');
% 
% xlabel('x','fontsize',12);
% ylabel('y','fontsize',12);
% zlabel('z','fontsize',12);
% axis([-5,5,-5,5,0,5]);
end

function dist = calculate_distance3(mat_start,mat_goal)
%% 此函数是计算三维点的距离的
dist = sqrt((mat_start(1)-mat_goal(1))^2+(mat_start(2)-mat_goal(2))^2+(mat_start(3)-mat_goal(3))^2);
end

function flag = collision_checking_cylinder(num_cylinder,dat_cylinder,coor_new,coor_near,Delta,deta)
%发生碰撞返回1
flag = 0;
 
for k1=1:num_cylinder
    x_coor = dat_cylinder{k1}(1);
    y_coor = dat_cylinder{k1}(2);
    z_coor = dat_cylinder{k1}(3);
    R = dat_cylinder{k1}(4)/2;
    height = dat_cylinder{k1}(5);
     
    for r=Delta/deta:0.1:Delta
        coor_mid = rrt_steer(coor_new,coor_near,r);
        if (((x_coor-coor_mid(1))^2+(y_coor-coor_mid(2))^2-R^2) < 0.1) && (z_coor<coor_mid(3)) && (coor_mid(3)<z_coor+height+0.1)
             flag = 1;
             break;
        end
    end
end
end

function dat_cylinder = cylinder_coor_deal(dat_xia)
%% 将16个圆柱障碍物直径和高度等参数放在元胞数组里，至于为什么存在元胞数组里面，没有为什么，个人爱好
lin = size(dat_xia,1);
dat_cylinder=cell(lin,1);
for k1=1:lin
    dat_cylinder{k1}=dat_xia(k1,:);
end
end

function path_best = delete_redundant_points(path,num_cylinder,dat_cylinder)
num_points = size(path,1);
count = 1;
start_point = path(1,:);
index = zeros(1,num_points);
for k1 = 1:num_points-2
    count = count+1;
    final_point = path(count+1,:);
    if  collision_checking_cylinder(num_cylinder,dat_cylinder,final_point,start_point,calculate_distance3(final_point,start_point),3)==1
      
        start_point = path(count,:);
    else
        index(count) = count;
    end
end
index(index==0) = [];
path([index(end:-1:1)],:) = [];
path_best = path;
end


function plot_cylinder(coor,diameter,height,facealpha,color)
 
%% plot_cylinder(dat_xia(k2,1:3),dat_xia(k2,4),dat_xia(k2,5),1,rand(1,3));
%  第一个参数是圆柱体的底部圆心坐标值，第二个参数是圆柱体直径，第三个参数是圆柱高度
%  第四个参数是透明度，第五个参数是颜色矩阵

%% 函数解释：把这个函数当做黑箱处理，只需要记住函数的输入就可以，知道是干什么的，内部实现过于复杂，很难解释清楚
 
% coor:         中心坐标
% diameter:     直径
% height:       高度
% facealpha:    透明度
% color:        颜色
 
r = diameter/2;
theta = 0:0.3:pi*2;
hold on
 
for k1 = 1:length(theta)-1
    X=[coor(1)+r*cos(theta(k1)) coor(1)+r*cos(theta(k1+1)) coor(1)+r*cos(theta(k1+1)) coor(1)+r*cos(theta(k1))];
    Y=[coor(2)+r*sin(theta(k1)) coor(2)+r*sin(theta(k1+1)) coor(2)+r*sin(theta(k1+1)) coor(2)+r*sin(theta(k1))];
    Z=[coor(3),coor(3),coor(3)+height,coor(3)+height];
    h=fill3(X,Y,Z,color);
    set(h,'edgealpha',0,'facealpha',facealpha)  
end
 
X=[coor(1)+r*cos(theta(end)) coor(1)+r*cos(theta(1)) coor(1)+r*cos(theta(1)) coor(1)+r*cos(theta(end))];
Y=[coor(2)+r*sin(theta(end)) coor(2)+r*sin(theta(1)) coor(2)+r*sin(theta(1)) coor(2)+r*sin(theta(end))];
Z=[coor(3),coor(3),coor(3)+height,coor(3)+height];
h=fill3(X,Y,Z,color);
set(h,'edgealpha',0,'facealpha',facealpha)
 
 
fill3(coor(1)+r*cos(theta),coor(2)+r*sin(theta),coor(3)*ones(1,size(theta,2)),color)
fill3(coor(1)+r*cos(theta),coor(2)+r*sin(theta),height+coor(3)*ones(1,size(theta,2)),color)
view(3)
end


function [coor_near,coor_index] = rrt_near(coor_rand,T)
 
min_distance = calculate_distance3(coor_rand,[T.x(1),T.y(1),T.z(1)]);
 
for T_iter=1:size(T.x,2)
    temp_distance=calculate_distance3(coor_rand,[T.x(T_iter),T.y(T_iter),T.z(T_iter)]);
    
    if temp_distance<=min_distance
        min_distance=temp_distance;
        coor_near(1)=T.x(T_iter);
        coor_near(2)=T.y(T_iter);
        coor_near(3)=T.z(T_iter);
        coor_index=T_iter;
    end
end

end

function coor_rand = rrt_sample(Pg,Map,goal,start)
 
if unifrnd(0,1)<Pg %随机的0-1连续均匀分布的数组
   coor_rand=goal;
else
   coor_rand(1)= unifrnd(-0.2,1.5)* Map(1);   
   coor_rand(2)= unifrnd(-0.2,1)* Map(2);   
   coor_rand(3)= unifrnd(-0.2,1)* Map(3);   
   coor_rand = coor_rand+start;
  
end
end

function coor_new = rrt_steer(coor_rand,coor_near,Delta)
 
 deltaX = coor_rand(1)-coor_near(1);
 deltaY = coor_rand(2)-coor_near(2);
 deltaZ = coor_rand(3)-coor_near(3);
 r = sqrt(deltaX^2+deltaY^2+deltaZ^2);
 fai = atan2(deltaY,deltaX);  
 theta = acos(deltaZ/r);
 
 R = Delta;
 x1 = R*sin(theta)*cos(fai);
 x2 = R*sin(theta)*sin(fai);
 x3 = R*cos(theta);
 
 coor_new(1) = coor_near(1)+x1;
 coor_new(2) = coor_near(2)+x2;
 coor_new(3) = coor_near(3)+x3;
 
end

function [dat_xia] = xlsData() 
 %% 前三列是十六个圆柱形障碍物底部圆心坐标，第四列是直径，第五列是高度
              
 dat_xia =  [
     1,-2.5,0,0.5,1.5;
     1,-2,0,0.5,1.5;
     1,-1.5,0,0.5,1.5;
     1,-1,0,0.5,1.5;
     1,-0.5,0,0.5,1.5;
     1,0,0,0.5,1.5;
     1,0.5,0,0.5,1.5;
     1,1,0,0.5,1.5;
     1,1.5,0,0.5,1.5;
     1,2,0,0.5,1.5 ;
     -2,-4.5,0,0.5,5;
     -2,-4,0,0.5,5;
     -2,-3.5,0,0.5,5;
     -2,-3,0,0.5,5;
     -2,-2.5,0,0.5,5;
     -2,-2,0,0.5,5;
     -2,-1.5,0,0.5,5;
     -2,1.5,0,0.5,5;
     -2,2,0,0.5,5;
     -2,2.5,0,0.5,5;
     -2,3,0,0.5,5;
     -2,3.5,0,0.5,5;
     -2,4,0,0.5,5;
     -2,4.5,0,0.5,5;
     ];
 
 
 
 
 
 

         
end