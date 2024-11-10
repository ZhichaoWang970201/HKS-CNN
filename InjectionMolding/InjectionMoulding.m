clear all
close all
clc

num_ex = 3000;

for i = 1:num_ex
    tic
    num_cut = randi([0,5],1);
    POINT = [];
    NORMAL = [];
    BB = zeros(num_cut,4);
    len = zeros(num_cut+1,1);
    
    % create internal cut
    if num_cut == 0
            POINT = [];
            NORMAL = [];
            len(1,1) = 0;
    else
        tran = zeros(num_cut,2); % the translation of each internal cut
        for j = 1:num_cut
            cut_shape = randi([1,10],1);
            FLAG = 2; % interior
            MIN = 1;
            MAX = 2;
        
            % point creation
            if cut_shape == 1 % circle
                r = rand(1)*(MAX-MIN) + MIN;
                [normal, point] = circ(r, FLAG);
            elseif cut_shape == 2 % ellipse
                a = rand(1)*(MAX-MIN) + MIN;
                b = rand(1)*(MAX-MIN) + MIN;
                [normal, point] = elli(a, b, FLAG); 
            else
                r = rand(1)*(MAX-MIN) + MIN;
                [normal, point] = poly(r, FLAG);
            end
        
            % collision check
            if j == 1
                BB(1,:) = Bound_Box(point); % the first one has no collision 
            else
                BB(j,:) = Bound_Box(point); % the first one has no collision
                index_x = 0; % initialization
                index_y = 0; % initialization
                while index_x==0 && index_y==0 % get rid of both both x and y translations are zero
                    index_x = randi([-1,1],1);
                    index_y = randi([-1,1],1);
                end
                x_tran = index_x*(rand(1)*(MAX-MIN) + MIN);
                y_tran = index_y*(rand(1)*(MAX-MIN) + MIN);
                flag_c = 1; % indicate that there is collision
                while flag_c == 1
                    index = collision_check(BB, x_tran, y_tran, j);
                    if index == 1 % collsion
                        x_tran = 1.2 * x_tran;
                        y_tran = 1.2 * y_tran;
                    else % no collision
                        flag_c = 2;
                        point(1,:) = point(1,:) + x_tran;
                        point(2,:) = point(2,:) + y_tran;
                        BB(j,1:2) = BB(j,1:2) + x_tran * ones(1,2); 
                        BB(j,3:4) = BB(j,3:4) + y_tran * ones(1,2); 
                        tran(j,:) = [x_tran, y_tran];
                    end
                end
            end
        
            % point and normal vectors inclusion 
            POINT = [POINT, point];
            NORMAL = [NORMAL, normal];
            len(j,1) = size(point,2);
        end
    end
    % create external boundary
    cut_shape = randi([1,10],1);
    FLAG = 1; % exterior
    MIN = 2;
    MAX = 4;
    
    % point creation
    if cut_shape == 1 % circle
        r = rand(1)*(MAX-MIN) + MIN;
        [normal, point] = circ(r, FLAG);
    elseif cut_shape == 2 % ellipse
        a = rand(1)*(MAX-MIN) + MIN;
        b = rand(1)*(MAX-MIN) + MIN;
        [normal, point] = elli(a, b, FLAG); 
    else
        r = rand(1)*(MAX-MIN) + MIN;
        [normal, point] = poly(r, FLAG);
    end
    
    center = [0 ; 0];
    if num_cut ~= 0
        % create interior bounding box
        BB_inter = zeros(1,4);
        BB_inter(1,1) = min(BB(:,1));
        BB_inter(1,2) = max(BB(:,2));
        BB_inter(1,3) = min(BB(:,3));
        BB_inter(1,4) = max(BB(:,4));
        center = [ (BB_inter(1,1)+BB_inter(1,2))/2 ; (BB_inter(1,3)+BB_inter(1,4))/2 ];
    
        % interior & exterior collision check
        flag_ie_c = 1;
        while flag_ie_c == 1
            index = collision_ie_check(BB_inter, point+center);
            if index == 1 % collision
                point = 1.2 * point;
            else
                flag_ie_c = 2;
            end
        end
    end
    POINT = [POINT, point+center];
    NORMAL = [NORMAL, normal];
    len(end,1) = size(point,2);
    
%     figure()
%     n_start = 1;
%     for k = 1:num_cut+1
%         point_plot_orig = POINT(:,n_start:n_start+len(k,1)-1);
%         n_start = n_start+len(k,1);
%         point_plot = [point_plot_orig, point_plot_orig(:,1)];
%         plot(point_plot(1,:), point_plot(2,:));
%         hold on        
%     end 
    
    % Mesh in 2D plane
    % create polygon shape object
    if num_cut == 0
        %T_new = delaunay(POINT(1,:),POINT(2,:));
        pgon_new = polyshape(POINT(1,:),POINT(2,:),'SolidBoundaryOrientation','ccw');
    else
        index = zeros(num_cut+1,2);
        for j = 1:num_cut+1
            if j == 1
                index(1,:) = [1, len(1,1)];
            else
                index(j,:) = [sum(len(1:j-1,1))+1,sum(len(1:j,1))];
            end
        end
        pgon_new = polyshape(POINT(1,index(end,1):index(end,2)),POINT(2,index(end,1):index(end,2)),'SolidBoundaryOrientation','ccw'); % may give some warning to get rid of near points
        for j = 1:num_cut
            pgon_old = polyshape(POINT(1,index(j,1):index(j,2)),POINT(2,index(j,1):index(j,2)),'SolidBoundaryOrientation','ccw'); % may give some warning to get rid of near points
            pgon_new = subtract(pgon_new, pgon_old); % can have holds 
        end
    end
    
    T_final = triangulation(pgon_new);
    V = T_final.Points;
    F = T_final.ConnectivityList;
    V1 = pgon_new.Vertices;
    
    nan_index = find(isnan(V1(:,1))==1);
    for j = 1:num_cut
        if j == 1
            len(1,1) = nan_index(1,1)-1;
        else
            len(j,1) = nan_index(j,1)-nan_index(j-1,1)-1;
        end
    end
    len(end,1) = size(V1,1)-nan_index(end,1);
    
    % change the sequence of points
    POINT = [V(len(1,1)+1:end,:)', V(1:len(1,1),:)'];
    % change the sequence of len
    len = [len(2:end,1); len(1,1)];
    % change the sequence of face index
    index = [[1:1:len(end,1)]+(size(POINT,2)-len(end,1)), 1:1:size(POINT,2)-len(end,1)];
    for j = 1:size(F,1)
        for k = 1:3
            T_new(j,k) = index(1,F(j,k));    
        end
    end   
    
%     figure()
%     trimesh(T_new,POINT(1,:),POINT(2,:));

%     % check the quality of T_new
%     T_NEW = T_new(:,[1:3,1]);
%     edge = [ min(T_NEW(1,1:2)),max(T_NEW(1,1:2)); min(T_NEW(1,2:3)),max(T_NEW(1,2:3)); min(T_NEW(1,3:4)),max(T_NEW(1,3:4)) ];
%     num_count = [1; 1; 1];
%     for row_idx = 2:size(T_NEW,1)
%         for col_idx = 1:3
%             edge_new = [min(T_NEW(row_idx,col_idx:col_idx+1)), max(T_NEW(row_idx,col_idx:col_idx+1))];
%             if ismember(edge_new, edge, 'rows') == 1
%                 edge_idx = find(ismember(edge, edge_new,'rows'));
%                 num_count(edge_idx,1) = num_count(edge_idx,1) + 1;
%             else
%                 edge = [edge; edge_new];
%                 num_count(size(edge,1),1) = 1;
%             end
%         end
%     end
    
    % get index of cut
    index = zeros(num_cut,2);
    for j = 1:num_cut
        if j == 1
            index(j,1) = 1;
            index(j,2) = len(j,1);
        else
            index(j,1) = sum(len(1:j-1,1))+1;
            index(j,2) = sum(len(1:j,1));
        end
    end

    % create translate in z axis
    x_dist = max(POINT(1,:)) - min(POINT(1,:));
    y_dist = max(POINT(2,:)) - min(POINT(2,:));
    z_max = min(x_dist, y_dist); % the largest thickness
    z = max(rand(1)*0.05,0.01) * z_max;
    
    point3D = [];
    normal3D = [];
    face3D = [];
    % bottom plane
    point_new = zeros(3,size(POINT,2));
    point_new(1:2,:) = POINT;
    normal_new = zeros(3,size(T_new,1));
    normal_new(3,:) = -ones(1,size(T_new,1));
    point3D = [point3D, point_new];
    normal3D = [normal3D, normal_new];
    face3D = [face3D; T_new];
    clear point_new normal_new;
    
    % top plane
    point_new = zeros(3,size(POINT,2));
    point_new(1:2,:) = POINT;
    point_new(3,:) = z*ones(1,size(POINT,2));
    normal_new = zeros(3,size(T_new,1));
    normal_new(3,:) = ones(1,size(T_new,1));
    point3D = [point3D, point_new];
    normal3D = [normal3D, normal_new];
    face3D = [face3D; T_new+size(POINT,2)*ones(size(T_new,1),3)];
    clear point_new normal_new;
    clear T_new;
    
    % construct outer surface boundary
    index_b = sum(len)-len(end,1)+1:1:sum(len);
    index_t = index_b+sum(len);
    ratio = rand(1)*0.04+1.01;
    point3D(:,index_b) = ratio*point3D(:,index_b); % extend the area of the bottom surface
    z_lateral = (rand(1)*0.4+0.1) * z_max;
    ratio_xy = randsrc(1,1, [1,rand(1)*0.5+1; 1/2,1/2]);
    point_new_b = point3D(:,index_b)*ratio_xy;
    point_new_b(3,:) = z_lateral;
    point_new_t = point3D(:,index_t)*ratio_xy;
    point_new_t(3,:) = z_lateral;
    index_b_new = [size(point3D,2)+1:1:size(point3D,2)+len(end,1)];
    index_t_new = [size(point3D,2)+len(end,1)+1:1:size(point3D,2)+2*len(end,1)];
    point3D = [point3D, point_new_b, point_new_t];
    index_lateral = zeros(4,len(end,1)+1);
    index_lateral(1,1:len(end,1)) = index_b;
    index_lateral(2,1:len(end,1)) = index_b_new;
    index_lateral(3,1:len(end,1)) = index_t_new;
    index_lateral(4,1:len(end,1)) = index_t;
    index_lateral(:,end) = index_lateral(:,1);
    for j = 1:len(end,1)
        for k = 1:3
            face3D = [face3D; index_lateral(k,j) index_lateral(k,j+1) index_lateral(k+1,j); index_lateral(k+1,j) index_lateral(k,j+1) index_lateral(k+1,j+1)];
            point = point3D(:,face3D(end,:));
            normal = cross(point(:,2)-point(:,1), point(:,3)-point(:,1));
            normal = normal/norm(normal);
            normal3D = [normal3D, normal, normal];
        end
    end
    
    % lateral surface
    for j = 1:num_cut
        clear index_b index_t;
        index_b = [index(j,1):1:index(j,2),index(j,1)];
        index_t = index_b + sum(len); % upper index
        fill_shape = randi(3);
        if fill_shape == 1 % fill by lateral surface
            for k = 1:len(j,1)
                face3D = [face3D ; index_b(1,k) index_b(1,k+1) index_t(1,k) ; index_t(1,k) index_b(1,k+1) index_t(1,k+1)];
                point = point3D(:,face3D(end,:));
                normal = cross(point(:,2)-point(:,1), point(:,3)-point(:,1));
                normal = normal/norm(normal);
                normal3D = [normal3D, normal, normal];
            end            
        elseif fill_shape == 2 % fill by partial lateral surface
            index_b = [index(j,1):1:index(j,2)];
            index_t = index_b + sum(len); % upper index
            ratio = rand(1)*0.5+0.4;
            point3D(:,index_b) = ratio*(point3D(:,index_b)-[tran(j,:),0]')+[tran(j,:),0]'; % extend the area of the bottom surface
            z_internal = (rand(1)*0.5+0.4) * z_lateral;
            ratio_xy = randsrc(1,1, [1,rand(1)*1/3+2/3; 1/2,1/2]);
            point_new_b = ratio_xy*(point3D(:,index_b)-[tran(j,:),0]')+[tran(j,:),0]';
            point_new_b(3,:) = z_internal;
            point_new_t = ratio_xy*(point3D(:,index_t)-[tran(j,:),0]')+[tran(j,:),0]';
            point_new_t(3,:) = z_internal;
            index_b = [index(j,1):1:index(j,2),index(j,1)];
            index_t = index_b + sum(len); % upper index
            index_b_new = [size(point3D,2)+1:1:size(point3D,2)+len(j,1),size(point3D,2)+1];
            index_t_new = [size(point3D,2)+len(j,1)+1:1:size(point3D,2)+2*len(j,1),size(point3D,2)+len(j,1)+1];
            point3D = [point3D, point_new_b, point_new_t];
            index_internal = zeros(4,len(j,1)+1);
            index_internal(1,:) = index_b;
            index_internal(2,:) = index_b_new;
            index_internal(3,:) = index_t_new;
            index_internal(4,:) = index_t;
            for ll = 1:len(j,1)
                for k = 1:3
                    face3D = [face3D; index_internal(k,ll) index_internal(k,ll+1) index_internal(k+1,ll); index_internal(k+1,ll) index_internal(k,ll+1) index_internal(k+1,ll+1)];
                    point = point3D(:,face3D(end,:));
                    normal = cross(point(:,2)-point(:,1), point(:,3)-point(:,1));
                    normal = normal/norm(normal);
                    normal3D = [normal3D, normal, normal];
                end
            end
        else % fill_shape == 3 % fully fill
            index_b = [index(j,1):1:index(j,2)];
            index_t = index_b + sum(len); % upper index
            ratio = rand(1)*0.5+0.4;
            point3D(:,index_b) = ratio*(point3D(:,index_b)-[tran(j,:),0]')+[tran(j,:),0]'; % extend the area of the bottom surface
            z_internal = (rand(1)*0.5+0.4) * z_lateral;
            ratio_xy = randsrc(1,1, [1,rand(1)*1/3+2/3; 1/2,1/2]);
            point_new_b = ratio_xy*(point3D(:,index_b)-[tran(j,:),0]')+[tran(j,:),0]';
            point_new_b(3,:) = z_internal;
            point_new_b = [point_new_b, [tran(j,:),z_internal]'];
            point_new_t = ratio_xy*(point3D(:,index_t)-[tran(j,:),0]')+[tran(j,:),0]';
            point_new_t(3,:) = point_new_t(3,:)+z_internal;
            point_new_t = [point_new_t, [tran(j,:),point_new_t(3,1)]'];
            index_b = [index(j,1):1:index(j,2),index(j,1)];
            index_b(2,:) = [size(point3D,2)+1:1:size(point3D,2)+len(j,1),size(point3D,2)+1];
            index_b(3,1) = size(point3D,2)+len(j,1)+1;
            index_t = [index(j,1):1:index(j,2),index(j,1)]+ sum(len);
            index_t(2,:) = [size(point3D,2)+len(j,1)+2:1:size(point3D,2)+2*len(j,1)+1,size(point3D,2)+len(j,1)+2];
            index_t(3,1) = size(point3D,2)+2*len(j,1)+2;
  
            point3D = [point3D, point_new_b, point_new_t];
            for ll = 1:len(j,1)
                %bottom
                face3D = [face3D; index_b(1,ll) index_b(1,ll+1) index_b(2,ll); index_b(2,ll) index_b(1,ll+1) index_b(2,ll+1)];
                point = point3D(:,face3D(end,:));
                normal = cross(point(:,2)-point(:,1), point(:,3)-point(:,1));
                normal = -normal/norm(normal);
                normal3D = [normal3D, normal, normal];
                %top
                face3D = [face3D; index_t(1,ll) index_t(1,ll+1) index_t(2,ll); index_t(2,ll) index_t(1,ll+1) index_t(2,ll+1)];
                point = point3D(:,face3D(end,:));
                normal = cross(point(:,2)-point(:,1), point(:,3)-point(:,1));
                normal = normal/norm(normal);
                normal3D = [normal3D, normal, normal];
                %bottom
                face3D = [face3D; index_b(2,ll) index_b(2,ll+1) index_b(3,1)];
                point = point3D(:,face3D(end,:));
                normal = cross(point(:,2)-point(:,1), point(:,3)-point(:,1));
                normal = -normal/norm(normal);
                normal3D = [normal3D, normal];
                % top
                face3D = [face3D; index_t(2,ll) index_t(2,ll+1) index_t(3,1)];
                point = point3D(:,face3D(end,:));
                normal = cross(point(:,2)-point(:,1), point(:,3)-point(:,1));
                normal = normal/norm(normal);
                normal3D = [normal3D, normal];
            end
        end
        
    end
    
%     trimesh(face3D,point3D(1,:),point3D(2,:),point3D(3,:));
    figure()
    scatter3(point3D(1,:),point3D(2,:),point3D(3,:),'b');
    hold on;
    for j = 1:size(face3D,1)
        id = zeros(1,4);
        id(1,1:3) = face3D(j,:);
        id(1,4) = face3D(j,1);
        point_id = point3D(:,id);
        fill3(point_id(1,:),point_id(2,:),point_id(3,:),'r');
        %plot3(point_id(1,:),point_id(2,:),point_id(3,:),'r');
        hold on;        
    end  
    axis equal;
    name = strcat(num2str(i));
    saveas(gcf,strcat(name,'.fig')); 
    saveas(gcf,strcat(name,'.jpg'));
    close all;
    name = strcat(num2str(i),'.mat');
    save(name,'point3D','face3D','normal3D'); 
    toc;
end

function index = collision_ie_check(inter, point)
index = 0; % initialize no collision

% make sure cover
if min(point(1,:))>= inter(1,1)
    index = 1;
end
if max(point(1,:))<= inter(1,2)
    index = 1;
end
if min(point(2,:))>= inter(1,3)
    index = 1;
end
if max(point(2,:))<= inter(1,4)
    index = 1;
end

% check collision
len = size(point,2);
Point = zeros(2,len+1);
Point(:,1:len) = point;
Point(:,end) = point(:,1);

Inter = zeros(2,5);
Inter(:,1) = [inter(1,1) ; inter(1,3)];
Inter(:,2) = [inter(1,2) ; inter(1,3)];
Inter(:,3) = [inter(1,2) ; inter(1,4)];
Inter(:,4) = [inter(1,1) ; inter(1,4)];
Inter(:,5) = [inter(1,1) ; inter(1,3)];
for k = 1:len
    for kk = 1:4
        A = [Inter(:,kk+1)-Inter(:,kk) Point(:,k)-Point(:,k+1)];
        b = Point(:,k)-Inter(:,kk);
        t = inv(A)*b;
        if t(1,1)>=0 && t(1,1)<=1 && t(2,1)>=0 && t(2,1)<=1
            index = 1;
            break;
        end
    end
end
end

function index = collision_check(BB, x_tran, y_tran, j)
BB(j,1:2) = BB(j,1:2) + x_tran*ones(1,2); 
BB(j,3:4) = BB(j,3:4) + y_tran*ones(1,2);
index = 0;
for k = 1:j-1
    if BB(j,1) > BB(k,2) || BB(j,2) < BB(k,1) || BB(j,3) > BB(k,4) || BB(j,4) < BB(k,3)
    else
        index = 1;
        break;
    end
end

end

function Point = Bound_Box(point)
x_min = min(point(1,:));
x_max = max(point(1,:));
y_min = min(point(2,:));
y_max = max(point(2,:));
Point = [x_min, x_max, y_min, y_max];
end

function [norm, POINT] = poly(r, FLAG)
% FLAG: 1--exterior 2--interior
% Cham: 1--no chamfer 2--chamfer

num_v = randi([3,10],1);
point = zeros(2,num_v+1);

theta = 0:2*pi/num_v:2*pi*(num_v-1)/num_v;
for j = 1:num_v
    point(1,j) = r * cos(theta(1,j));
    point(2,j) = r * sin(theta(1,j));    
end
point(:,num_v+1) = point(:,1);

% sample two points in each line for chamfer
Point = zeros(2,3*num_v);
for j = 1:num_v
    kesi = rand(1)*0.5;
    lamda = rand(1)*0.5+0.5;
    Point(:,3*j-2) = point(:,j);
    Point(:,3*j-1) = point(:,j) + kesi*(point(:,j+1)-point(:,j));
    Point(:,3*j) = point(:,j) + lamda*(point(:,j+1)-point(:,j));
end
% rearrange Point
Point = [Point(:,end), Point(:,1:end-1)];

% chamfer
POINT = [];
cham =  randsrc(1,1, [1,2; 4/5,1/5]);
for j = 1:num_v
    if cham == 1
        POINT = [POINT, Point(:,3*j-2:3*j)];
    else
        POINT_new = Cham(Point(:,3*j-2:3*j));
        POINT = [POINT, POINT_new];        
    end  
end
norm = norm_cal(POINT, FLAG);
end

function [POINT_new] = Cham(Point)
point1 = Point(:,1);
point2 = Point(:,2);
point3 = Point(:,3);
u = 0:0.05:1;
F1 = (1-u).^2;
F2 = 2*(1-u).*u;
F3 = u.^2;
POINT_new = point1*F1 + point2*F2 + point3*F3;
end

function [norm, point] = circ(r, FLAG)
% FLAG: 1--exterior 2--interior

num = 36; % number of points in a circle is 36 / sample every 10 degree
for i = 1:num
   ang = 10*i*(pi/180);
   point(1,i)= (r*cos(ang));
   point(2,i)= (r*sin(ang));
end

norm = norm_cal(point, FLAG);

% point1 = [point(:,2:end) , point(:,1)];
% point1 = (point+point1)/2;
% figure()
% plot(point(1,:),point(2,:),'b');
% hold on
% scatter(point1(1,:),point1(2,:),'r');
% point2 = point1 + norm/10;
% hold on
% scatter(point2(1,:),point2(2,:),'m');
end

function [norm, point] = elli(a, b, FLAG)
% FLAG: 1--exterior 2--interior

num = 36; % number of points in a circle is 36 / sample every 10 degree
for i = 1:num
   ang = 10*i*(pi/180);
   point(1,i)= (a*cos(ang));
   point(2,i)= (b*sin(ang));
end

norm = norm_cal(point, FLAG);

% point1 = [point(:,2:end) , point(:,1)];
% point1 = (point+point1)/2;
% figure()
% plot(point(1,:),point(2,:),'b');
% hold on
% scatter(point1(1,:),point1(2,:),'r');
% point2 = point1 + norm/10;
% hold on
% scatter(point2(1,:),point2(2,:),'m');
end

function [normal] = norm_cal(point, FLAG)
% FLAG: 1--exterior 2--interior
len = size(point,2);
normal = zeros(2,len);
for i = 1:len
    if i == len
        dir = (point(:,1)-point(:,len)) / norm(point(:,1)-point(:,len));
        normal(:,i) = [0 1 ; -1 0] * dir;
    else
        dir = (point(:,i+1)-point(:,i)) / norm(point(:,i+1)-point(:,i));
        normal(:,i) = [0 1 ; -1 0] * dir;
    end
end
% interior change direction
if FLAG == 2
    normal = -normal;
end
end
