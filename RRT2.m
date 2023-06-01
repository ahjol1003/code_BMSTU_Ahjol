clc
clear 
close all;


%% set search range 
search_range=[10 10 10];
%% set start_point and goal

start_point=[1 3 0.5];%x,y -5
goal=[9 5 1];%x,y-5

%% 创造障碍物
[dat_xia] = xlsData();
[Dat_xia] = Data();
 
%数据重处理
num_cylinder = size(dat_xia,1);                            %圆柱体障碍物个数
dat_cylinder = cylinder_coor_deal(dat_xia);                %将圆柱体数据存放在元胞数组里，方便访问
 
    r=dat_xia(:,4);
    min_obs_radius = min(r);
%% 算法部分（rrt*-smart）
step_length=0.01;
Pg=0.5;                   %采样概率
max_fail_attemps=500;%最大失败次数=迭代次数
target_path_num=1;
tic
[path_RRTstar,path_RRTsmart,tree,treeS] = rrt_start_smart(Pg,start_point,goal,search_range,num_cylinder,dat_cylinder,min_obs_radius,step_length,max_fail_attemps,target_path_num);
toc
%% 绘图
plot_world(Dat_xia,start_point,goal,path_RRTstar,path_RRTsmart,tree,treeS);

%% 函数

function plot_world(Dat_xia,start_point,goal,path_RRTstar,path_RRTsmart,tree,treeS)

for k2=1:size(Dat_xia,1)
    plot_cylinder(Dat_xia(k2,1:3),Dat_xia(k2,4),Dat_xia(k2,5),1,rand(1,3));%这个是画圆柱体障碍物的函数
end
axis([-5,5,-5,5,0,5])  %设置图像的可视化范围
axis equal     % 图像坐标轴可视化间隔相等 
bar1 = scatter3(start_point(1)-5,start_point(2)-5,start_point(3),80,"cyan",'filled','o','MarkerEdgeColor','k');hold on
bar2 = scatter3(goal(1)-5,goal(2)-5,goal(3),80,"magenta",'filled',"o",'MarkerEdgeColor','k');
    text(start_point(1)-5,start_point(2)-5,start_point(3),'   Отправная точка'); text(goal(1)-5,goal(2)-5,goal(3),'   Конечная точка');
    xlabel('x','fontsize',12);
    ylabel('y','fontsize',12);
    zlabel('z','fontsize',12);
   axis([-5,5,-5,5,0,5])
    title('Трехмерное пространство планирования пути','fontsize',12);
       
    grid on;
    axis equal;
    
    %plot tree
    if size(tree,1)>1
       plot_tree(tree);
    end
    
    %plot best path
    
    if length(path_RRTstar)>0
        end_node=tree(size(tree,1),:);
        plot_path(end_node,tree,1,'b');
    end
    
    if length(path_RRTsmart)>0
        end_node=treeS(size(treeS,1),:);   
          branch = [end_node];
         parent_idx=end_node(4);
    while parent_idx>0
       node = treeS(parent_idx,:);
       branch = [node;branch];
       parent_idx=node(4);
    end
    if size(branch,1)>1
        X = branch(:,1)-5;
        Y = branch(:,2)-5;
        Z = branch(:,3);
       bar3= plot3(X,Y,Z,'-o','Color','r','MarkerSize',4,'MarkerFaceColor','k');
        axis([-5,5,-5,5,0,5])
        hold on;
        n=size(branch,1);
f(1) = 0;
for i = 2:n
    f(i) = f(i-1) +calculate_distance3(branch(i-1,1:3),branch(i,1:3));
end
f(n)
    end
    end
legend([bar1,bar2,bar3],["Отправная точка","Конечная точка","Траектория движения конечных точек роботизированных рук"])
end
function plot_tree(tree)
    idx = size(tree,1);
    while idx>1        
        node = tree(idx,:);
        plot_path(node,tree,0.2,'g');
        idx = idx - 1;
    end
end
function plot_path(end_node,tree,line_width,line_color)
    %end_node
    branch = [end_node];
    parent_idx=end_node(4);
    while parent_idx>0
       node = tree(parent_idx,:);
       branch = [node;branch];
       parent_idx=node(4);
    end
    if size(branch,1)>1
        X = branch(:,1)-5;
        Y = branch(:,2)-5;
        Z = branch(:,3);
        p = plot3(X,Y,Z);
        set(p,'Color',line_color,'LineWidth',line_width,'Marker','.','MarkerEdgeColor',line_color);
        axis([-5,5,-5,5,0,5])
        hold on;
    end
    
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
function [path_RRTstar,path_RRTsmart,tree,treeS] = rrt_start_smart(Pg,start_point,goal,search_range,num_cylinder,dat_cylinder,min_obs_radius,step_length,max_fail_attemps,target_path_num)
     start_node=[start_point 0 0 0 0 1];
     end_node=[goal -1 -1 0 0 -1];
     tree =[start_node];
     if norm(start_point-goal)>step_length
         fail_attemps = 0;
         path_num=0;
         % extend tree延伸树
         while 1

             [valid,is_last,tree]=extend_tree(Pg,tree,goal,min_obs_radius,num_cylinder,dat_cylinder,step_length,search_range);
             if is_last
                 path_num = path_num+1;             
             end
             
             if valid
                 fail_attemps =0;
             else
                fail_attemps = fail_attemps+1;      
             end

             %end
             if fail_attemps > max_fail_attemps
                 break
             end

             if path_num >= target_path_num
                %update end_node id
                 end_node(8)=size(tree,1)+1;
                 tree=[tree;end_node];
                 break
             end
         end

         % update_end_node_parent
         tree = update_end_node_parent (tree);
         path_RRTstar = get_path_from_tree(tree);
        
         % straighten best path  让路径变直
         treeS=tree(:,:);
         treeS = straighten_path(tree,min_obs_radius,num_cylinder,dat_cylinder,step_length);
         path_RRTsmart = get_path_from_tree(treeS);
   
    
     end
end
function [valid,is_last,tree]=extend_tree(Pg,tree,goal,min_obs_radius,num_cylinder,dat_cylinder,step_length,search_range)
    is_last=0;

    % find new point and check whether it is valid
    [valid,new_point,parent_idx,min_total_dist] = find_new_point(Pg,goal,tree,num_cylinder,dat_cylinder,min_obs_radius,step_length,search_range);

    % update its parent and near points' (major difference between rrt* and rrt) 重新选择父节点和临近点
    if valid 
        [parent_idx,min_total_dist,near_points] = find_better_parent_idx(new_point,min_total_dist,parent_idx,min_obs_radius,num_cylinder,dat_cylinder,step_length,tree);
    end
    
    % add new point to the tree if valid
    if valid
        [new_node_idx,is_last,tree] = add_node(new_point,parent_idx,min_total_dist,tree,goal,step_length);
    end
    
    if valid
        tree = update_new_node_parent_idx(tree,new_node_idx,step_length,min_total_dist,parent_idx,min_obs_radius,num_cylinder,dat_cylinder,near_points);
    end

end
function [valid,new_point,parent_idx,min_total_dist] = find_new_point(Pg,goal,tree,num_cylinder,dat_cylinder,min_obs_radius,step_length,search_range)
    %random point
     if rand <Pg
          sample_point =goal;
     else
       sample_point = rand(1,3).*search_range;
     end

    %find the node closest to new_point_tmp找到最接近new_point_tmp的节点
     distance = cal_leafs_to_point_distance(tree,sample_point);
     [~,parent_idx]=min(distance);
     closest_point = tree(parent_idx,1:3);
     % get new_point with step_length
     moving_directions = [sample_point(1)-closest_point(1) sample_point(2)-closest_point(2) sample_point(3)-closest_point(3)];%延伸方向
     moving_directions = moving_directions/sqrt(sum(moving_directions.^2));
     new_point = closest_point + step_length*moving_directions;%新点
     new_point_line_dist = norm(new_point-closest_point);
     
     
     if new_point_valid(new_point,num_cylinder,dat_cylinder,step_length,search_range) && link_points_valid(new_point,parent_idx,tree,min_obs_radius,num_cylinder,dat_cylinder,step_length)  %新点及新点连线都不碰撞和出界
         min_total_dist=tree(parent_idx,5)+new_point_line_dist;
         valid=1;
     else
         min_total_dist=0;
         valid=0;
     end

end
function [parent_idx,min_total_dist,near_points] = find_better_parent_idx(new_point,min_total_dist,parent_idx,min_obs_radius,num_cylinder,dat_cylinder,step_length,tree)
    %find near points
    search_radius=step_length*2;
    
    dist_mat = cal_leafs_to_point_distance(tree,new_point);
    near_points = find(dist_mat <= search_radius);
    
    for i = 1:length(near_points)
        idx = near_points(i);
        
        leaf_point = tree(idx,1:3);
        total_dist = tree(idx,5) + norm(leaf_point-new_point);
       if total_dist<min_total_dist && link_points_valid(new_point,parent_idx,tree,min_obs_radius,num_cylinder,dat_cylinder,step_length)
           min_total_dist=total_dist;
           parent_idx=idx;
       end
        
    end  


end
function tree=update_new_node_parent_idx(tree,new_node_idx,step_length,min_total_dist,parent_idx,min_obs_radius,num_cylinder,dat_cylinder,near_points)
    % nodes close to the new node could get a shorter path with new node as its parent
    total_dist_of_new_point=tree(new_node_idx,5);
    new_point = tree(new_node_idx,1:3);
    for i = 1:length(near_points)
        idx = near_points(i);
        near_point = tree(idx,1:3);
        pre_total_dist = tree(idx,5);
        new_total_dist = norm(near_point-new_point)+total_dist_of_new_point;
        
       if new_total_dist<pre_total_dist && link_points_valid(new_point,parent_idx,tree,min_obs_radius,num_cylinder,dat_cylinder,step_length)
           tree(idx,4)=new_node_idx; %parent
           tree(idx,5)=new_total_dist; %total_dist
           tree(idx,6)=0; % it is not the lastest anymore if it was
       end
    end

end
function [idx,is_last,tree] = add_node(new_point,parent_idx,min_total_dist,tree,goal,step_length)
  %add new node to tree
    if found_last_node_before_goal(new_point,goal,step_length)
        is_last = 1;
    else
        is_last =0;
    end
    id = size(tree,1)+1;
    new_node = [new_point parent_idx min_total_dist is_last 0 id];
    tree = [tree;new_node];  
    idx=size(tree,1);
end
function valid = new_point_valid(new_point,num_cylinder,dat_cylinder,step_length,search_range)
    valid = 1;
    done = 0;
    
    if valid && out_of_range(new_point,search_range)
        valid=0;
        done =1;
    end
    
    if valid && will_collide(new_point,num_cylinder,dat_cylinder,step_length)
        valid=0;
        done =1;
    end
end  %出界（搜索范围）或者碰撞到障碍物valid设为0，done=1   
function valid = link_points_valid(new_point,parent_idx,tree,min_obs_radius,num_cylinder,dat_cylinder,step_length)  %点到点的连线有没有碰撞
    done =0;
    valid =1;
  parent_point = tree(parent_idx,1:3);     
  if ~done && ~vaild_path(new_point,parent_point,min_obs_radius,num_cylinder,dat_cylinder,step_length)
        valid=0;             
        done =1;
  end
end
function valid = vaild_path(point1,point2,min_obs_radius,num_cylinder,dat_cylinder,step_length)
    valid = 1;
    num = floor(norm(point1-point2)/(min_obs_radius/2));%floor将x中元素取整，norm(point1-point2)范数，就是求点1和点2之间的距离
    x_step = (point2(1)-point1(1))/num;
    y_step = (point2(2)-point1(2))/num;
    z_step = (point2(3)-point1(3))/num;
    done = 0;
    for i = 1:num
        if ~done
            point = [point1(1)+x_step*i point1(2)+y_step*i point1(3)+z_step*i] ;
            if will_collide(point,num_cylinder,dat_cylinder,step_length)
                valid=0;
                done =1;
            end
        end
     end
        
end
function distance = cal_leafs_to_point_distance(tree,point)
    a = tree(:,1:3);
    b = ones(size(tree,1),1)*point;
    diff = a-b;
    sqrt_diff = diff.*diff;
    sum=zeros(size(sqrt_diff,1),1);
    
    for i =1:3
        sum=sum+sqrt_diff(:,i);
    end
    
    distance=sum.^0.5;
   
end
function result = found_last_node_before_goal(new_point,goal,step_length)
    distance = norm(new_point-goal);
    if distance <= step_length
        result = 1;
    else
        result = 0;
    end
end
function tree = update_end_node_parent (tree)
    candidates=[];
    for idx = 1:size(tree,1)
        if tree(idx,6)==1
            candidates = [candidates;tree(idx,:)];
        end
    end
    
    if size(candidates,1)>0    
        %find the last_node (not goal)  with min cost
        [~,idx]=min(candidates(:,5),[],2);
        winner_id=candidates(idx,8);
        
        %update end_node data
        end_node_id=size(tree,1);
        tree(end_node_id,4) = winner_id;
        tree(end_node_id,5) = norm(tree(end_node_id,1:3)-tree(winner_id,1:3)) + tree(winner_id,5);
    end
end
function path = get_path_from_tree(tree)
    end_node_idx=size(tree,1);
    %put last_node to path
        path=[end_node_idx];
        parent_idx=tree(end_node_idx,4);
        % add points
        while parent_idx > 0
            path=[parent_idx;path];
            parent_idx=tree(parent_idx,4);
        end
end
function tree = straighten_path(tree,min_obs_radius,num_cylinder,dat_cylinder,step_length)

    end_node_idx=size(tree,1);

    while 1
        jump_step=2;
        
        current_idx=end_node_idx;
        success_num=0;
        while 1 
           parent_idx = get_parent_idx(tree,current_idx,jump_step);
           if parent_idx ==0
                break;
           end 
           
           new_point=tree(current_idx,1:3);
           if link_points_valid(new_point,parent_idx,tree,min_obs_radius,num_cylinder,dat_cylinder,step_length)
               tree(current_idx,4)=parent_idx;
               current_idx=parent_idx;
               jump_step=jump_step*2;
               success_num=success_num+1;
           elseif jump_step>2
               jump_step=max(floor(jump_step/2),2);
           else
               current_idx = tree(current_idx,4);
           end

        end
        
        if success_num ==0
            break;
        end
        
    end
    
    
    



end
function idx = get_parent_idx(tree,idx,jump_step)
    if idx > 0
        for i = 1:jump_step
           idx= tree(idx,4);
           if idx ==0
               break;
           end
        end
    end
    
end
function result = out_of_range(point,search_range)
    result =0;
    if point(1)>search_range(1) || point(1)<0 || ...
       point(2)>search_range(2) || point(2)<0 || ...
       point(3)>search_range(3) || point(3)<0
       result = 1;
    end
end
function result = will_collide(point,num_cylinder,dat_cylinder,step_length)
    %发生碰撞返回1
result  = 0;
 
for k1=1:num_cylinder
    x_coor = dat_cylinder{k1}(1);
    y_coor = dat_cylinder{k1}(2);
    z_coor = dat_cylinder{k1}(3);
    R = dat_cylinder{k1}(4)/2;
    height = dat_cylinder{k1}(5);
     
    for r=step_length/3:0.1:step_length
        if (((x_coor-point(1))^2+(y_coor-point(2))^2-R^2) < 0.01) && (z_coor<point(3)) && (point(3)<z_coor+height+0.02)
             result  = 1;
             break;
        end
    end
end
end
function dist = calculate_distance3(mat_start,mat_goal)
%% 此函数是计算三维点的距离的
dist = sqrt((mat_start(1)-mat_goal(1))^2+(mat_start(2)-mat_goal(2))^2+(mat_start(3)-mat_goal(3))^2);
end
function dat_cylinder = cylinder_coor_deal(dat_xia)
%% 将16个圆柱障碍物直径和高度等参数放在元胞数组里，至于为什么存在元胞数组里面，没有为什么，个人爱好
lin = size(dat_xia,1);
dat_cylinder=cell(lin,1);
for k1=1:lin
    dat_cylinder{k1}=dat_xia(k1,:);
end
end
function [dat_xia] = xlsData() 
 %% 前三列是十六个圆柱形障碍物底部圆心坐标，第四列是直径，第五列是高度
              %x,y+5
 dat_xia =  [
     6,2.5,0,0.5,1.5;
     6,3,0,0.5,1.5;
     6,3.5,0,0.5,1.5;
     6,4,0,0.5,1.5;
     6,4.5,0,0.5,1.5;
     6,5,0,0.5,1.5;
     6,5.5,0,0.5,1.5;
     6,6,0,0.5,1.5;
     6,6.5,0,0.5,1.5;
     6,7,0,0.5,1.5 ;
     3,0.5,0,0.5,5;
     3,1,0,0.5,5;
     3,1.5,0,0.5,5;
     3,2,0,0.5,5;
     3,2.5,0,0.5,5;
     3,3,0,0.5,5;
     3,3.5,0,0.5,5;
     3,6.5,0,0.5,5;
     3,7,0,0.5,5;
     3,7.5,0,0.5,5;
     3,8,0,0.5,5;
     3,8.5,0,0.5,5;
     3,9,0,0.5,5;
     3,9.5,0,0.5,5;
     ];        
end
function [Dat_xia] = Data() 
 %% 前三列是十六个圆柱形障碍物底部圆心坐标，第四列是直径，第五列是高度
              %x,y+5
 Dat_xia =  [
   
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
