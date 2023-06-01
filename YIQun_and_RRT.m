clear 
clc
close all

%% 导入数据
citys=[
 -3.5,2.5,2.7;
 -0.5,-4.5,2.2;
 1.25,4,2.7;
 3,1,1.7;
 2.5,4,2.7;
 -4.5,-1,0.7;
 -4.5,-3,1.2;
 -4,-2,0.7;
 0.5,-4.5,3.2;
 -0,4,2.9;
 
  -5,-2,0.7;
 2,0,0.7;
 3,-1,2.7;
 3.5,0,1.2;
 -4,1,2.7;
 -3,-1,1.2;
 -1,1,2.2;
 -4.5,2.5,2.9;
4,0,6.2;5.5,0,6.2;-5.5,0,1.7;-7,0,1.7;-7,0,1;];
  




[dat_xia] = xlsData();
%% 数据重处理
num_cylinder = size(dat_xia,1);                            %圆柱体障碍物个数
dat_cylinder = cylinder_coor_deal(dat_xia);                %将圆柱体数据存放在元胞数组里，方便访问；dat_cylinder: 圆柱体障碍物数据元胞
%% 计算城市间相互距离
fprintf('Computing Distance Matrix... \n');
n = size(citys,1);
D = zeros(n,n);
% v=1;
for i = 1:n
    for j = 1:n
        if i ~= j
                     
% rrt算法部分
PATH = [];
goal =citys(i,:);
start =citys(j,:);
path_best = RRT(num_cylinder,dat_cylinder,start,goal);%每次找到最好路径存放在这个变量里
%注意：这个路径点是经过冗余点处理以后的，所以点很少
% Point(:,v:v+2)=path_best;
% v=v+3;
WWW= size(path_best,1);
d=zeros(1,WWW-1);
 for kkk=1:WWW-1
 d(1,kkk)= sqrt(sum((path_best(kkk,:)-path_best(kkk+1,:) ).^2));
 end
D(i,j)=sum(d)
        else
            D(i,j) = 1e-4;      %0
        end
    end    
end

%% 初始化参数
fprintf('Initializing Parameters... \n');
m = 50;                              % 蚂蚁数量
alpha =1;                           % 信息素重要程度因子
beta = 2;                            % 启发函数重要程度因子
rho = 0.3;                           % 信息素挥发因子
Q = 1000;                               % 常系数
Eta = 1./D;                          % 启发函数
Tau = ones(n,n);                     % 信息素矩阵
Table = zeros(m,n);                  % 路径记录表
iter = 1;                            % 迭代次数初值
iter_max = 300;                      % 最大迭代次数 
Route_best = zeros(iter_max,n);      % 各代最佳路径       
Length_best = zeros(iter_max,1);     % 各代最佳路径的长度  
Length_ave = zeros(iter_max,1);      % 各代路径的平均长度  

%% 迭代寻找最佳路径
figure(1)

for k2=1:size(dat_xia,1)
    plot_cylinder(dat_xia(k2,1:3),dat_xia(k2,4),dat_xia(k2,5),1,rand(1,3));%这个是画圆柱体障碍物的函数
end
axis([-7,7,-7,7,0,10])
axis equal     % 图像坐标轴可视化间隔相等
grid on   
hold on 
while iter <= iter_max
    fprintf('迭代第%d次\n',iter);
    % 随机产生各个蚂蚁的起点城市
      start = zeros(m,1);
      for i = 1:m
          temp = randperm(n);
          start(i) = temp(1);
      end
      Table(:,1) = start; 
      % 构建解空间
      citys_index = 1:n;
      % 逐个蚂蚁路径选择
      for i = 1:m
          % 逐个城市路径选择
         for j = 2:n
             tabu = Table(i,1:(j - 1));           % 已访问的城市集合(禁忌表)
             allow_index = ~ismember(citys_index,tabu);
             allow = citys_index(allow_index);  % 待访问的城市集合
             P = allow;
             % 计算城市间转移概率
             for k = 1:length(allow)
                 P(k) = Tau(tabu(end),allow(k))^alpha * Eta(tabu(end),allow(k))^beta;
             end
             P = P/sum(P);
             % 轮盘赌法选择下一个访问城市
             Pc = cumsum(P);     
            target_index = find(Pc >= rand); 
            target = allow(target_index(1));
            Table(i,j) = target;
         end
     
      end
      
      
      
      % 计算各个蚂蚁的路径距离
      Length = zeros(m,1);
      for i = 1:m
          Route = Table(i,:);
          for j = 1:(n - 1)
              Length(i) = Length(i) + D(Route(j),Route(j + 1));
          end
          Length(i) = Length(i) + D(Route(n),Route(1));
      end
      % 计算最短路径距离及平均距离
      if iter == 1
          [min_Length,min_index] = min(Length);
          Length_best(iter) = min_Length;  
          Length_ave(iter) = mean(Length);
          Route_best(iter,:) = Table(min_index,:);
      else
          [min_Length,min_index] = min(Length);
          Length_best(iter) = min(Length_best(iter - 1),min_Length);%前后2次迭代的min比较
          Length_ave(iter) = mean(Length);
          %最优值先输入表中
          if Length_best(iter) == min_Length
              Route_best(iter,:) = Table(min_index,:);
          else
              Route_best(iter,:) = Route_best((iter-1),:);
          end
      end
      % 更新信息素
      Delta_Tau = zeros(n,n);
      % 逐个蚂蚁计算
      for i = 1:m
          % 逐个城市计算
          for j = 1:(n - 1)
              Delta_Tau(Table(i,j),Table(i,j+1)) = Delta_Tau(Table(i,j),Table(i,j+1)) + Q/Length(i);
          end
          Delta_Tau(Table(i,n),Table(i,1)) = Delta_Tau(Table(i,n),Table(i,1)) + Q/Length(i);
      end
      Tau = (1-rho) * Tau + Delta_Tau;
  

   
%最佳路径的迭代变化过程
    [Shortest_Length,index] = min(Length_best(1:iter));
    Shortest_Route = Route_best(index,:);
   % 迭代次数加1，清空路径记录表
    iter = iter + 1;
    Table = zeros(m,n);


end
%% 结果显示
[Shortest_Length,index] = min(Length_best);
Shortest_Route = Route_best(index,:);
% disp(['Кратчайшее расстояние:' num2str(Shortest_Length)]);
disp(['Оптимальный путь:' num2str([Shortest_Route Shortest_Route(1)])]);

%% 绘图

hold on
bar1=plot3([citys(Shortest_Route,1);citys(Shortest_Route(1),1)],[citys(Shortest_Route,2);citys(Shortest_Route(1),2)],[citys(Shortest_Route,3);citys(Shortest_Route(1),3)],'o-','LineWidth',2,'Color','k');
for i = 1:size(citys,1)
    text(citys(i,1),citys(i,2),citys(i,3),['        Точка' num2str(i)],'FontSize',8,'FontWeight','Bold','Color','k');
end

for i = 1:size(citys,1)
    uu=Shortest_Route(i);
    text(citys(uu,1),citys(uu,2),citys(uu,3),['  (' num2str(i),')'],'FontSize',8,'FontWeight','Bold','Color','k');
end
xlabel('x','fontsize',12);
ylabel('y','fontsize',12);
zlabel('z','fontsize',12);
title(['Алгоритм муравьиной колонии (Доступ к кратчайшему порядку всех точек пути:' num2str([Shortest_Route Shortest_Route(1)]) ')']);
hold on
bar2 = scatter3(citys(Shortest_Route(1),1),citys(Shortest_Route(1),2),citys(Shortest_Route(1),3),60,"cyan",'filled','o','MarkerEdgeColor','r','MarkerFaceColor','k');
hold on
bar3 = scatter3(citys(Shortest_Route(end),1),citys(Shortest_Route(end),2),citys(Shortest_Route(end),3),60,"magenta",'filled',"o",'MarkerEdgeColor','r','MarkerFaceColor','k');
axis([-7,7,-7,7,0,10])
textstr1={'                          ';'                          '; '【Отправная точка】'};
text(citys(Shortest_Route(1),1),citys(Shortest_Route(1),2),citys(Shortest_Route(1),3),textstr1,'FontSize',8,'FontWeight','Bold','Color','k');
textstr2={'                          ';'                          '; '【Конечная точка】'};
text(citys(Shortest_Route(end),1),citys(Shortest_Route(end),2),citys(Shortest_Route(end),3),textstr2,'FontSize',8,'FontWeight','Bold','Color','k');

%% RRT路径
hold on

 goal1 =citys(Shortest_Route(1),:);
 start1 =citys(Shortest_Route(2),:);
path_best1 = RRTT(num_cylinder,dat_cylinder,start1,goal1);
n1=size(path_best1,1);
path_best(1:n1,:)=path_best1;

goal2 =citys(Shortest_Route(2),:);
start2 =citys(Shortest_Route(3),:);
path_best2 = RRTT(num_cylinder,dat_cylinder,start2,goal2);
n2=size(path_best2,1);
path_best(n1+1:n1+n2,:)=path_best2;

goal3 =citys(Shortest_Route(3),:);
start3 =citys(Shortest_Route(4),:);
path_best3 = RRTT(num_cylinder,dat_cylinder,start3,goal3);
n3=size(path_best3,1);
path_best(n1+n2+1:n1+n2+n3,:)=path_best3;

goal4=citys(Shortest_Route(4),:);
start4 =citys(Shortest_Route(5),:);
path_best4 = RRTT(num_cylinder,dat_cylinder,start4,goal4);
n4=size(path_best4,1);
path_best(n1+n2+n3+1:n1+n2+n3+n4,:)=path_best4;

goal5=citys(Shortest_Route(5),:);
start5 =citys(Shortest_Route(6),:);
path_best5 = RRTT(num_cylinder,dat_cylinder,start5,goal5);
n5=size(path_best5,1);
path_best(n1+n2+n3+n4+1:n1+n2+n3+n4+n5,:)=path_best5;

goal6 =citys(Shortest_Route(6),:);
start6 =citys(Shortest_Route(7),:);
path_best6 = RRTT(num_cylinder,dat_cylinder,start6,goal6);
n6=size(path_best6,1);
path_best(n1+n2+n3+n4+n5+1:n1+n2+n3+n4+n5+n6,:)=path_best6;

goal7 =citys(Shortest_Route(7),:);
start7 =citys(Shortest_Route(8),:);
path_best7 = RRTT(num_cylinder,dat_cylinder,start7,goal7);
n7=size(path_best7,1);
path_best(n1+n2+n3+n4+n5+n6+1:n1+n2+n3+n4+n5+n6+n7,:)=path_best7;

goal8 =citys(Shortest_Route(8),:);
start8 =citys(Shortest_Route(9),:);
path_best8 = RRTT(num_cylinder,dat_cylinder,start8,goal8);
n8=size(path_best8,1);
path_best(n1+n2+n3+n4+n5+n6+n7+1:n1+n2+n3+n4+n5+n6+n7+n8,:)=path_best8;

goal9 =citys(Shortest_Route(9),:);
start9 =citys(Shortest_Route(10),:);
path_best9 = RRTT(num_cylinder,dat_cylinder,start9,goal9);
n9=size(path_best9,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+1:n1+n2+n3+n4+n5+n6+n7+n8+n9,:)=path_best9;

goal10 =citys(Shortest_Route(10),:);
start10 =citys(Shortest_Route(11),:);
path_best10 = RRTT(num_cylinder,dat_cylinder,start10,goal10);
n10=size(path_best10,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10,:)=path_best10;

goal11 =citys(Shortest_Route(11),:);
start11 =citys(Shortest_Route(12),:);
path_best11 = RRTT(num_cylinder,dat_cylinder,start11,goal11);
n11=size(path_best11,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11,:)=path_best11;

goal12 =citys(Shortest_Route(12),:);
start12 =citys(Shortest_Route(13),:);
path_best12 = RRTT(num_cylinder,dat_cylinder,start12,goal12);
n12=size(path_best12,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12,:)=path_best12;

goal13 =citys(Shortest_Route(13),:);
start13 =citys(Shortest_Route(14),:);
path_best13 = RRTT(num_cylinder,dat_cylinder,start13,goal13);
n13=size(path_best13,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13,:)=path_best13;

goal14 =citys(Shortest_Route(14),:);
start14 =citys(Shortest_Route(15),:);
path_best14 = RRTT(num_cylinder,dat_cylinder,start14,goal14);
n14=size(path_best14,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14,:)=path_best14;


goal15 =citys(Shortest_Route(15),:);
start15 =citys(Shortest_Route(16),:);
path_best15 = RRTT(num_cylinder,dat_cylinder,start15,goal15);
n15=size(path_best15,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15,:)=path_best15;

goal16 =citys(Shortest_Route(16),:);
start16 =citys(Shortest_Route(17),:);
path_best16 = RRTT(num_cylinder,dat_cylinder,start16,goal16);
n16=size(path_best16,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16,:)=path_best16;

goal17 =citys(Shortest_Route(17),:);
start17 =citys(Shortest_Route(18),:);
path_best17 = RRTT(num_cylinder,dat_cylinder,start17,goal17);
n17=size(path_best17,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17,:)=path_best17;

goal18 =citys(Shortest_Route(18),:);
start18 =citys(Shortest_Route(19),:);
path_best18 = RRTT(num_cylinder,dat_cylinder,start18,goal18);
n18=size(path_best18,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18,:)=path_best18;

goal19 =citys(Shortest_Route(19),:);
start19 =citys(Shortest_Route(20),:);
path_best19 = RRTT(num_cylinder,dat_cylinder,start19,goal19);
n19=size(path_best19,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+n19,:)=path_best19;

goal20 =citys(Shortest_Route(20),:);
start20 =citys(Shortest_Route(21),:);
path_best20 = RRTT(num_cylinder,dat_cylinder,start20,goal20);
n20=size(path_best20,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+n19+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+n19+n20,:)=path_best20;

goal21 =citys(Shortest_Route(21),:);
start21 =citys(Shortest_Route(22),:);
path_best21 = RRTT(num_cylinder,dat_cylinder,start21,goal21);
n21=size(path_best21,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+n19+n20+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+n19+n20+n21,:)=path_best21;

goal22 =citys(Shortest_Route(22),:);
start22 =citys(Shortest_Route(23),:);
path_best22 = RRTT(num_cylinder,dat_cylinder,start22,goal22);
n22=size(path_best22,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+n19+n20+n21+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+n19+n20+n21+n22,:)=path_best22;

goal23 =citys(Shortest_Route(23),:);
start23 =citys(Shortest_Route(1),:);
path_best23 = RRTT(num_cylinder,dat_cylinder,start23,goal23);
n23=size(path_best23,1);
path_best(n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+n19+n20+n21+n22+1:n1+n2+n3+n4+n5+n6+n7+n8+n9+n10+n11+n12+n13+n14+n15+n16+n17+n18+n19+n20+n21+n22+n23,:)=path_best23;


hold on 
bar4=plot3(path_best(:,1),path_best(:,2),path_best(:,3),'--','Color','r','MarkerSize',4,'MarkerFaceColor','r','LineWidth',3);
axis([-7,7,-7,7,0,10]);
legend([bar1,bar2,bar3,bar4],["Кратчайший путь доступа","Отправная точка","Конечная точка","Лучший путь избегания препятствий"])

figure(2)
bar1=plot3([citys(Shortest_Route,1);citys(Shortest_Route(1),1)],[citys(Shortest_Route,2);citys(Shortest_Route(1),2)],[citys(Shortest_Route,3);citys(Shortest_Route(1),3)],'o-','LineWidth',2,'Color','k');
for i = 1:size(citys,1)
    text(citys(i,1),citys(i,2),citys(i,3),['        Точка' num2str(i)],'FontSize',8,'FontWeight','Bold','Color','k');
end

for i = 1:size(citys,1)
    uu=Shortest_Route(i);
    text(citys(uu,1),citys(uu,2),citys(uu,3),['  (' num2str(i),')'],'FontSize',8,'FontWeight','Bold','Color','k');
end
xlabel('x','fontsize',12);
ylabel('y','fontsize',12);
zlabel('z','fontsize',12);
title(['Алгоритм муравьиной колонии (Доступ к кратчайшему порядку всех точек пути:' num2str([Shortest_Route Shortest_Route(1)]) ')']);



figure(3)
plot(1:iter_max,Length_best,'b',1:iter_max,Length_ave,'r:','linewidth',1.5)
legend('Кратчайший путь','Средний путь')
xlabel('Количество итераций')
ylabel('Длина пути')
grid on
title('Кратчайший и Средний пути')











%% 函数
function path_best = RRT(num_cylinder,dat_cylinder,start,goal)
%% 流程初始化
 
% Delta=1;                % 设置扩展步长，扩展结点允许的最大距离，这个数据越大迭代越快，但是比最优解效果越差
max_iter = 10000;        % 最大迭代次数，如果超过这个次数还没找到路径则认为找不到路径
Map = [goal(1)-start(1),5,5];
%Map = [goal(1)-start(1),goal(2)-start(2),goal(3)-start(3)];

count = 1;
 
%% 构建初始化树
T.x(1) = start(1);
T.y(1) = start(2);
T.z(1) = start(3);
T.xpar(1) = goal(1);
T.ypar(1) = goal(2);
T.zpar(1) = goal(3);
T.dist(1) = 0;
T.indpre(1) = 0;
 

for iter = 1:max_iter
 
    % step1: 在地图上随机采样
    coor_rand = rrt_sample(Map,goal,start);     %在空间进行随机采样，coor_rand是一个  1×3  的数组
    % plot3(coor_rand(1),coor_rand(2),coor_rand(3),'r*')
    
    % step2 : 遍历树，找到最近的父节点
    [coor_near,coor_index] = rrt_near(coor_rand,T);
         for k1=1:num_cylinder
    x_coor = dat_cylinder{k1}(1);
    y_coor = dat_cylinder{k1}(2);
    z_coor = dat_cylinder{k1}(3);
    R = dat_cylinder{k1}(4)/2;
    height = dat_cylinder{k1}(5);
        if (((x_coor-coor_near(1))^2+(y_coor-coor_near(2))^2-R^2) <0.5) && (coor_near(3)<z_coor+height+0.5)
            Delta=0.2;
        else
           Delta=1;
        end
      end
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
%     line([coor_near(1),coor_new(1)],[coor_near(2),coor_new(2)],[coor_near(3),coor_new(3)],'LineWidth',1,'color','g');
%  axis([-5,5,-5,5,0,10]);
%     pause(0.000001); %暂停0.1s，使得RRT扩展过程容易观察；
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
%         line([goal(1),coor_new(1)],[goal(2),coor_new(2)],[goal(3),coor_new(3)],'LineWidth',1,'color','g');
%      axis([-7,7,-7,7,0,10])
%         pause(0.000001); %暂停0.01s，使得RRT扩展过程容易观察
        break;
    end
 
    
end

 
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
end
function path_best = RRTT(num_cylinder,dat_cylinder,start,goal)
%% 流程初始化
 
% Delta=0.05;                % 设置扩展步长，扩展结点允许的最大距离，这个数据越大迭代越快，但是比最优解效果越差
max_iter = 10000;        % 最大迭代次数，如果超过这个次数还没找到路径则认为找不到路径
Map = [goal(1)-start(1),5,5];
%Map = [goal(1)-start(1),goal(2)-start(2),goal(3)-start(3)];
count = 1;
 
%% 构建初始化树
T.x(1) = start(1);
T.y(1) = start(2);
T.z(1) = start(3);
T.xpar(1) = goal(1);
T.ypar(1) = goal(2);
T.zpar(1) = goal(3);
T.dist(1) = 0;
T.indpre(1) = 0;
 

for iter = 1:max_iter
 
    % step1: 在地图上随机采样
    coor_rand = rrt_sample(Map,goal,start);     %在空间进行随机采样，coor_rand是一个  1×3  的数组
    % plot3(coor_rand(1),coor_rand(2),coor_rand(3),'r*')
    
    % step2 : 遍历树，找到最近的父节点
    [coor_near,coor_index] = rrt_near(coor_rand,T);
          for k1=1:num_cylinder
    x_coor = dat_cylinder{k1}(1);
    y_coor = dat_cylinder{k1}(2);
    z_coor = dat_cylinder{k1}(3);
    R = dat_cylinder{k1}(4)/2;
    height = dat_cylinder{k1}(5);
        if (((x_coor-coor_near(1))^2+(y_coor-coor_near(2))^2-R^2) <2) && (coor_near(3)<z_coor+height+2)
            Delta=0.01;
        else
           Delta=0.1;
            end
        end
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
     line([coor_near(1),coor_new(1)],[coor_near(2),coor_new(2)],[coor_near(3),coor_new(3)],'LineStyle','-','LineWidth',2,'color','g');
  axis([-7,7,-7,7,0,10]);
    pause(0.00000000000000000000001); %暂停0.1s，使得RRT扩展过程容易观察；
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
         line([goal(1),coor_new(1)],[goal(2),coor_new(2)],[goal(3),coor_new(3)],'LineStyle','-','LineWidth',2,'color','g');
      axis([-7,7,-7,7,0,10])
       pause(0.00000000000000000000000001); %暂停0.01s，使得RRT扩展过程容易观察
        break;
    end
 
    
end

 
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
        if (((x_coor-coor_mid(1))^2+(y_coor-coor_mid(2))^2-R^2) < 0.02) && (z_coor<coor_mid(3)) && (coor_mid(3)<z_coor+height+0.02)
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

%% 函数解释：把这个函数当做黑箱处理，只需要记住函数的输入就可以，知道是干什么的，内部
%% 实现过于复杂，很难解释清楚
 
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
function coor_rand = rrt_sample(Map,goal,start)
 
if unifrnd(0,1)<0.5
   coor_rand(1)= unifrnd(-0.2,1.5)* Map(1);   
   coor_rand(2)= unifrnd(-0.2,1)* Map(2);   
   coor_rand(3)= unifrnd(-0.2,1)* Map(3);   
   coor_rand = coor_rand+start;
else
   coor_rand=goal;
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
     %最左边
     
     -6,-4.75,0,0.5,4.5;
     -6,-4.25,0,0.5,4.5;
     -6,-3.75,0,0.5,4.5;
     -6,-3.25,0,0.5,4.5;
     -6,-2.75,0,0.5,4.5;
     -6,-2.25,0,0.5,4.5;
     -6,-1.75,0,0.5,4.5;
     -6,-1.25,0,0.5,4.5;
     -6,-0.75,0,0.5,4.5;
     
     -6,4.75,0,0.5,4.5;
     -6,4.25,0,0.5,4.5;
     -6,3.75,0,0.5,4.5;
     -6,3.25,0,0.5,4.5;
     -6,2.75,0,0.5,4.5;
     -6,2.25,0,0.5,4.5;
     -6,1.75,0,0.5,4.5;
     -6,1.25,0,0.5,4.5;
     -6,0.75,0,0.5,4.5;
     
     
     -6,0.25,0,0.5,1.5;
     -6,-0.25,0,0.5,1.5;
 

   %最右边
     4.75,-6.75,0,0.5,10;
     4.75,-6.25,0,0.5,10;
     4.75,-5.75,0,0.5,10;
     4.75,-5.25,0,0.5,10;
     4.75,-4.75,0,0.5,10;
     4.75,-4.25,0,0.5,10;
   
     4.75,-3.75,0,0.5,10;
     4.75,-3.25,0,0.5,10;
     4.75,-2.75,0,0.5,10;
     4.75,-2.25,0,0.5,10;
     4.75,-1.75,0,0.5,10;
     4.75,-1.25,0,0.5,10;
     4.75,-0.75,0,0.5,10;
     
     4.75,6.75,0,0.5,10;
     4.75,6.25,0,0.5,10;
     4.75,5.75,0,0.5,10;
     4.75,5.25,0,0.5,10;
     4.75,4.75,0,0.5,10;
     4.75,4.25,0,0.5,10;
     
     4.75,3.75,0,0.5,10;
     4.75,3.25,0,0.5,10;
     4.75,2.75,0,0.5,10;
     4.75,2.25,0,0.5,10;
     4.75,1.75,0,0.5,10;
     4.75,1.25,0,0.5,10;
     4.75,0.75,0,0.5,10;
  
     
     4.75,0.25,0,0.5,6;
     4.75,-0.25,0,0.5,6;
 
   %上下
   0,-6.9,0,0.2,10;
   0,-6.7,0,0.2,10;
   0,-6.5,0,0.2,10;
   0,-6.3,0,0.2,10;
   0,-6.1,0,0.2,10;
   0,-5.9,0,0.2,10;
   0,-5.7,0,0.2,10;
   0,-5.5,0,0.2,10;
   0,-5.3,0,0.2,10;
   0,-5.1,0,0.2,10;
   0,-4.9,0,0.2,10;
   0,-4.7,0,0.2,10;
   0,-4.5,0,0.2,10;
   0,-4.3,0,0.2,10;
   0,-4.1,0,0.2,10;
   0,-3.9,0,0.2,10;
   0,-3.7,0,0.2,10;

   
   
   2,5.9,0,0.2,3;
   2,5.7,0,0.2,3;
   2,5.5,0,0.2,3;
   2,5.3,0,0.2,3;
   2,5.1,0,0.2,3;
   2,4.9,0,0.2,3;
   2,4.7,0,0.2,3;
   2,4.5,0,0.2,3;
   2,4.3,0,0.2,3;
   2,4.1,0,0.2,3;
   2,3.9,0,0.2,3;
   2,3.7,0,0.2,3;
   2,3.5,0,0.2,3;
   2,3.3,0,0.2,3;
   2,3.1,0,0.2,3;
   2,2.9,0,0.2,3;
   2,2.7,0,0.2,3;
   2,2.5,0,0.2,3;
   2,2.3,0,0.2,3;
   2,2.1,0,0.2,3;
   

   0.5,5.9,0,0.2,3;
   0.5,5.7,0,0.2,3;
   0.5,5.5,0,0.2,3;
   0.5,5.3,0,0.2,3;
   0.5,5.1,0,0.2,3;
   0.5,4.9,0,0.2,3;
   0.5,4.7,0,0.2,3;
   0.5,4.5,0,0.2,3;
   0.5,4.3,0,0.2,3;
   0.5,4.1,0,0.2,3;
   0.5,3.9,0,0.2,3;
   0.5,3.7,0,0.2,3;
   0.5,3.5,0,0.2,3;
   0.5,3.3,0,0.2,3;
   0.5,3.1,0,0.2,3;
   0.5,2.9,0,0.2,3;
   0.5,2.7,0,0.2,3;
   0.5,2.5,0,0.2,3;
   0.5,2.3,0,0.2,3;
   0.5,2.1,0,0.2,3;
  %迷宫


 -3,1,0,0.1,3;
 -3,1.1,0,0.1,3;
 -3,1.2,0,0.1,3;
 -3,1.3,0,0.1,3;
 -3,1.4,0,0.1,3;
 -3,1.5,0,0.1,3;
 -3,1.6,0,0.1,3;
 -3,1.7,0,0.1,3;
 -3,1.8,0,0.1,3;
 -3,1.9,0,0.1,3;
 -3,2,0,0.1,3;
 -3,2.1,0,0.1,3;
 -3,2.2,0,0.1,3;
 -3,2.3,0,0.1,3;
 -3,2.4,0,0.1,3;
 -3,2.5,0,0.1,3;
 -3,2.6,0,0.1,3;
 -3,2.7,0,0.1,3;
 -3,2.8,0,0.1,3;
 -3,2.9,0,0.1,3;
 -3,3,0,0.1,3;
 
  -4,3,0,0.1,3;
  -3.9,3,0,0.1,3;
  -3.8,3,0,0.1,3;
  -3.7,3,0,0.1,3;
  -3.6,3,0,0.1,3;
  -3.5,3,0,0.1,3;
  -3.4,3,0,0.1,3;
  -3.3,3,0,0.1,3;
  -3.2,3,0,0.1,3;
  -3.1,3,0,0.1,3;
  -3,3,0,0.1,3;
  -2.9,3,0,0.1,3;
  -2.8,3,0,0.1,3;
  -2.7,3,0,0.1,3;
  -2.6,3,0,0.1,3;
  -2.5,3,0,0.1,3;
  -2.4,3,0,0.1,3;
  -2.3,3,0,0.1,3;
  -2.2,3,0,0.1,3;
  -2.1,3,0,0.1,3;
  -2,3,0,0.1,3;
  

  
  -4,4,0,0.1,3;
  -4,3.9,0,0.1,3;
  -4,3.8,0,0.1,3;
  -4,3.7,0,0.1,3;
  -4,3.6,0,0.1,3;
  -4,3.5,0,0.1,3;
  -4,3.4,0,0.1,3;
  -4,3.3,0,0.1,3;
  -4,3.2,0,0.1,3;
  -4,3.1,0,0.1,3;
  -4,3,0,0.1,3;
  -4,2.9,0,0.1,3;
  -4,2.8,0,0.1,3;
  -4,2.7,0,0.1,3;
  -4,2.6,0,0.1,3;
  -4,2.5,0,0.1,3;
  -4,2.4,0,0.1,3;
  -4,2.3,0,0.1,3;
  -4,2.2,0,0.1,3;
  -4,2.1,0,0.1,3;
  -4,2,0,0.1,3;
  
  -5,2,0,0.1,3;
  -4.9,2,0,0.1,3;
  -4.8,2,0,0.1,3;
  -4.7,2,0,0.1,3;
  -4.6,2,0,0.1,3;
  -4.5,2,0,0.1,3;
  -4.4,2,0,0.1,3;
  -4.3,2,0,0.1,3;
  -4.2,2,0,0.1,3;
  -4.1,2,0,0.1,3;
  -4,2,0,0.1,3;
  -3.9,2,0,0.1,3;
  -3.8,2,0,0.1,3;
  -3.7,2,0,0.1,3;
  -3.6,2,0,0.1,3;
  -3.5,2,0,0.1,3;
  -3.4,2,0,0.1,3;
  -3.3,2,0,0.1,3;
  -3.2,2,0,0.1,3;
  -3.1,2,0,0.1,3;
  -3,2,0,0.1,3;
    %中间

     
 -3.5,2.5,0,0.6,2.5;
 -0.5,-4.5,0,0.4,2;
 1.25,4,0,0.4,2.5;
 3,1,0,0.4,1.5;
 2.5,4,0,0.4,2.5;
 -4.5,-1,0,0.4,0.5;
 -4.5,-3,0,0.4,1;
 -4,-2,0,0.4,0.5;
 0.5,-4.5,0,0.4,3;
 0,4,0,0.4,2.7;
 
 
 -5,-2,0,0.4,0.5;
 2,0,0,0.3,0.5;
 3,-1,0,0.3,2.5;
 3.5,0,0,0.4,1;
 -4,1,0,0.4,2.5;
 -3,-1,0,0.3,1;
 -1,1,0,0.4,2;
 -4.5,2.5,0,0.2,2.7;

 
     ];
 
         
end