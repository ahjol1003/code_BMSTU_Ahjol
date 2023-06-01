%% 旅行商问题(TSP)优化
%% 清空环境变量
clear 
clc
close all

%% 导入数据
citys=[1,3;4,5;8,7;2.4,3;5,15;5,2;25,25;10,15;16,32;20,25;25,20;20,20;11,6;16,20;24,25;3.5,6;1,3.2;4.2,3;10,1;9,4;4,2;3,2;14,2;1.2,4;2,4;4,6;8,34;6,9;5,21;1,22;3,12;3,15.4;];
tic
%% 计算城市间相互距离
fprintf('Computing Distance Matrix... \n');
n = size(citys,1);
D = zeros(n,n);
for i = 1:n
    for j = 1:n
        if i ~= j
            D(i,j) = sqrt(sum((citys(i,:) - citys(j,:)).^2));
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
rho = 0.2;                           % 信息素挥发因子
Q = 100;                               % 常系数
Eta = 1./D;                          % 启发函数
Tau = ones(n,n);                     % 信息素矩阵
Table = zeros(m,n);                  % 路径记录表
iter = 1;                            % 迭代次数初值
iter_max = 100;                      % 最大迭代次数 
Route_best = zeros(iter_max,n);      % 各代最佳路径       
Length_best = zeros(iter_max,1);     % 各代最佳路径的长度  
Length_ave = zeros(iter_max,1);      % 各代路径的平均长度  

%% 迭代寻找最佳路径
figure;
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
  

 %   figure;
 %最佳路径的迭代变化过程
    [Shortest_Length,index] = min(Length_best(1:iter));
    Shortest_Route = Route_best(index,:);
    plot([citys(Shortest_Route,1);citys(Shortest_Route(1),1)],...
    [citys(Shortest_Route,2);citys(Shortest_Route(1),2)],'o-');
    pause(0.3);
   % 迭代次数加1，清空路径记录表
    iter = iter + 1;
    Table = zeros(m,n);


end

%% 结果显示
[Shortest_Length,index] = min(Length_best);
Shortest_Route = Route_best(index,:);
disp(['Кратчайшее расстояние:' num2str(Shortest_Length)]);
disp(['Кратчайший путь:' num2str([Shortest_Route Shortest_Route(1)])]);
toc
%% 绘图
figure(1)
plot([citys(Shortest_Route,1);citys(Shortest_Route(1),1)],...
     [citys(Shortest_Route,2);citys(Shortest_Route(1),2)],'o-','LineWidth',3);
grid on
for i = 1:size(citys,1)
    text(citys(i,1),citys(i,2),['   ' num2str(i)],'FontSize',15);
end
hold on
bar2 = scatter(citys(Shortest_Route(1),1),citys(Shortest_Route(1),2),80,"cyan",'filled','o','MarkerEdgeColor','r','MarkerFaceColor','k');
hold on
bar3 = scatter(citys(Shortest_Route(end),1),citys(Shortest_Route(end),2),80,"magenta",'filled',"o",'MarkerEdgeColor','r','MarkerFaceColor','k');

text(citys(Shortest_Route(1),1),citys(Shortest_Route(1),2),'       Отправная точка','FontSize',20);
text(citys(Shortest_Route(end),1),citys(Shortest_Route(end),2),'       Конечная точка','FontSize',20);
xlabel('X')
ylabel('Y')
title(['Алгоритм муравьиной колонии для оптимизации путей(Кратчайшее расстояние:' num2str(Shortest_Length) ')'],'FontSize',20)
figure(2)
% plot(1:iter_max,Length_best,'b',1:iter_max,Length_ave,'r:')
% legend('Кратчайший путь','Средний путь')
% xlabel('Количество итераций')
% ylabel('Длина пути')
% title('α=1')
plot(1:iter_max,Length_best,'b','LineWidth',3)
legend('Кратчайший путь','FontSize',20)
xlabel('Количество итераций','FontSize',20)
ylabel('Длина пути','FontSize',20)
% axis([0 50 26 27]);
 title('Процесс итерации оптимального пути','FontSize',20);