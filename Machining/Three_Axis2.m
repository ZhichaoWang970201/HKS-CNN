clear all
close all
clc

num_ex = 500;

for i = 501:1200
    tic
    num_cut = randi([0,7],1);
    pgon_old = polyshape();
    
    % create internal cut
    if num_cut == 0
        % do nothing here
    else
        for j = 1:num_cut
            cut_shape = randi([1,10],1);
            MIN = 1;
            MAX = 2;
            XY_LIM = 4*MAX;
        
            % point creation
            if cut_shape == 1 % circle
                r = rand(1)*(MAX-MIN) + MIN;
                [point] = circ(r);
            elseif cut_shape == 2 % ellipse
                r = rand(1)*(MAX-MIN) + MIN;
               [point] = elli(r); 
            else
                r = rand(1)*(MAX-MIN) + MIN;
                [point] = poly(r);
            end
            
            % translate in x and y direction
            tran = (rand(2,1)-0.5)*XY_LIM;
            point = point+tran;
            
            % create polygon shape object
            pgon_new = polyshape(point(1,:),point(2,:),'SolidBoundaryOrientation','ccw'); % may give some warning to get rid of near points
            % boolean operation
            pgon_old = union(pgon_new, pgon_old);
        end
    end
    
    % remove the possible holes
    pgon_old = rmholes(pgon_old);
   
    % create external boundary
    cut_shape = randi([1,10],1);
    MIN = 5;
    MAX = 10;
    
    % point creation
    if cut_shape == 1 % circle
        r = rand(1)*(MAX-MIN) + MIN;
        [point] = circ(r);
    elseif cut_shape == 2 % ellipse
        r = rand(1)*(MAX-MIN) + MIN;
        [point] = elli(r); 
    else
        r = rand(1)*(MAX-MIN) + MIN;
        [point] = poly(r);
    end
    pgon_new = polyshape(point(1,:),point(2,:),'SolidBoundaryOrientation','ccw');
%     figure()
%     plot(pgon_new)
%     figure()
%     plot(pgon_old)
    
    flag_region = 1;
    while flag_region == 1
        pgon_final = subtract(pgon_new, pgon_old); % can have holds
        if pgon_final.NumRegions == 1 % cannot have two regions
            flag_region = 2;
        else
            tran = (rand(2,1)-0.5)*XY_LIM;
            pgon_old = translate(pgon_old, tran(1,1), tran(2,1));
        end
    end
    T_final = triangulation(pgon_final);
%     figure()
%     plot(pgon_final)
%     figure()
%     triplot(T_final);
    V = T_final.Points;
    F = T_final.ConnectivityList;
    V1 = pgon_final.Vertices;
    
    % calculate the length of each shape
    index = find(isnan(V1(:,1))==1);
    if size(index,1) == 0 % no hole
        len = size(V,1);        
    else
        len = zeros(size(index,1)+1,1);
        for j = 1:size(index,1)+1
            if j == 1
                len(j,1) = index(1,1)-1;
            elseif j == size(index,1)+1
                len(j,1) = size(V1,1)-index(j-1,1);
            else
                len(j,1) = index(j,1)-index(j-1,1)-1;
            end
        end        
    end
    % calculate the area of each shape
    area = zeros(size(len,1),1); % area
    index_s = zeros(size(len,1),1); % start index
    index_e = zeros(size(len,1),1); % end index
    num = 0; % counter
    for j = 1:size(len,1)
        index_s(j,1) = num+1;
        index_e(j,1) = num+len(j,1);
        area(j,1) = polyarea(V(index_s(j,1):index_e(j,1),1),V(index_s(j,1):index_e(j,1),2)); % calculate the area covered by the shape
        num = num+len(j,1);
    end  
    FLAG = (area == max(area)); % index 1---exterior/ 0----interior
    
    % calculate normal vector
    N = [];
    for j = 1:size(len,1)
        [N_new] = norm_cal(V(index_s(j,1):index_e(j,1),:)', FLAG(j,1));
        N = [N, N_new];
    end
        
    % to make my new code consistent with my old code
    POINT = V';
    T_new = F;
    NORMAL = N;
    
    
    % create translate in z axis
    x_dist = max(POINT(1,:)) - min(POINT(1,:));
    y_dist = max(POINT(2,:)) - min(POINT(2,:));
    z_max = min(x_dist, y_dist); % the largest thickness
    z = (rand(1)*0.5+0.2) * z_max;
    
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
    
    % lateral surface
    for j = 1:size(len,1)
        point_list1 = zeros(1,len(j,1)+1); % bottom index
        point_list2 = zeros(1,len(j,1)+1); % top index
        normal_list = zeros(3,len(j,1)); % normal vector

        % index and corresponging normal    
        point_list1 = index_s(j,1):1:index_e(j,1);
        point_list2 = point_list1 + size(POINT,2);    
        point_list1(1,len(j,1)+1) = point_list1(1,1); % go back to the first to form a circle
        point_list2(1,len(j,1)+1) = point_list2(1,1); % go back to the first to form a circle
        normal_list(1:2,1:len(j,1)) = NORMAL(:,index_s(j,1):1:index_e(j,1)); % get corresponding normal vectors
        
        for k = 1:len(j,1)
            face_new  = [point_list1(1,k) point_list1(1,k+1) point_list2(1,k) ; point_list1(1,k+1) point_list2(1,k) point_list2(1,k+1)];
            normal_new = [normal_list(:,k) , normal_list(:,k)];
            face3D = [face3D; face_new];
            normal3D = [normal3D, normal_new];
        end
        
    end
%     trimesh(face3D,point3D(1,:),point3D(2,:),point3D(3,:));

    % sample 0 to 3 triangle for add or subtract
    % calculate the area of each triangle
    num_lateral = randi([0,3],1);
    num_lateral = min(num_lateral,size(F,1)); % depends on the largest number of face
    
    if num_lateral ~= 0
        Area = zeros(1,size(F,1));
        for j = 1:size(F,1)
            face_index = face3D(j+size(F,1),:);
            point_cood = point3D(:,face_index);
            Area(1,j) = 0.5*norm(cross(point_cood(:,2)-point_cood(:,1), point_cood(:,3)-point_cood(:,1)));
        end
        [sort_Area, face_index] = sort(Area,'descend');
        face_index = face_index+size(F,1);
    
        for j = 1:num_lateral
            point_i = face3D(face_index(1,j),:); % point_index
            point_as = point3D(:,point_i); % point coodinate for addition or subtraction
            [point_c, r_max] = center_cal(point_as); 
 
            FLAG_as = 1-binornd(1,0.8); %1: addition; %0: subtraction
            [norm_lateral, point_lateral, face_lateral] = extrude_3D(r_max, z);
        
            if FLAG_as == 0
                point_lateral(3,:) = -point_lateral(3,:);
                norm_lateral(3,:) = - norm_lateral(3,:);
                norm_lateral = - norm_lateral;
            end
            point_lateral = point_lateral + point_c;
            face_lateral = face_lateral + size(point3D,2); % update face_lateral
        
            point3D = [point3D, point_lateral];
            face3D = [face3D; face_lateral];
            normal3D = [normal3D, norm_lateral];
        
            % add new face within the triangle (with cut)
            new_point_index = find(point_lateral(3,:)==point_c(3,1));
            point_new = [point_as, point_lateral(:,new_point_index)];
            point_new_index = [point_i, size(point3D,2)-size(point_lateral,2)+new_point_index];
            face_new_orig = delaunay(point_new(1,:),point_new(2,:));
            face_new = [];
            for k = 1:size(face_new_orig,1)
                if min(face_new_orig(k,:))>3
                else
                    face_new = [face_new ; face_new_orig(k,:)];
                end            
            end
            for k = 1:size(face_new,1)
                face_new(k,:) = point_new_index(face_new(k,:));            
            end 
            norm_new = [0;0;1]*ones(1,size(face_new,1));
            face3D = [face3D ; face_new];
            normal3D = [normal3D, norm_new];
        end
    
        % delete the repeated surface
        index_total = 1:1:size(face3D,1);
        index_del = face_index(1,1:num_lateral);
        index_left = setdiff(index_total, index_del);
        face3D = face3D(index_left,:);
        normal3D = normal3D(:,index_left);
    end
    
    % internal cut for lateral surface
    index_ex = [index_s(find(FLAG==1),1):1:index_e(find(FLAG==1),1)];
    index_in = [setdiff([1:1:sum(len)], index_ex), [2*sum(len)+1:1:size(point3D,2)]];
    point_ex = point3D(1:2,index_ex);
    point_in = point3D(1:2,index_in);
    % calculate the length of exteral side
    point_ex = [point_ex, point_ex(:,1)];
    dist = zeros(1,size(point_ex,2)-1);
    for j = 1:size(point_ex,2)-1
        dist(1,j) = norm(point_ex(:,j)-point_ex(:,j+1));
    end
    [sort_dist, index_d] = sort(dist,'descend');
    point_line = point_ex(:,[index_d(1,1),index_d(1,1)+1]);
    z_xy_max = dist_pl(point_line, point_in);
    
    index_f = randi([1,3],1);
    point_i = [index_ex(1,index_d(1,1)), index_ex(1,index_d(1,1))+1];
    if index_ex(1,index_d(1,1)) == index_ex(1,end) % it correspond the last line--go back to the first line
        point_i = [index_ex(1,index_d(1,1)), index_ex(1,1)];
    end
    face_id = intersect(find(face3D(:,1)==point_i(1,1)), find(face3D(:,2)==point_i(1,2)));
    if index_f == 1
        num_f = face_id;
    elseif index_f == 2
        num_f = face_id+1;
    else
        num_f = [face_id, face_id+1];
    end
    
    % cut surface
    for j = 1:size(num_f,2)
        point_i = face3D(num_f(1,j),:); % point_index
        point_as = point3D(:,point_i); % point coodinate for addition or subtraction
        [point_c, r_max] = center_cal(point_as);
        if size(z_xy_max,2) == 0 % no cut internal---return error -corresponding to line 261: z_xy_max = dist_pl(point_line, point_in);
            z_xy_max = r_max; 
        end
        [norm_lateral, point_lateral, face_lateral] = extrude_3D(r_max, z_xy_max);
        norm_lateral = - norm_lateral;
        index_bottom = find(point_lateral(3,:)==0);
        
        a = [0;0;-1];
        b = normal3D(:,num_f(1,j));
        R = rotate_3D(a,b); % ratation_matrix
        point_lateral = R*point_lateral;
        point_lateral = point_lateral + point_c;
        norm_lateral = R*norm_lateral;
        % concate point, face and normal
        num_s = size(point3D,2);
        face3D = [face3D; face_lateral+num_s];
        point3D = [point3D, point_lateral];
        normal3D = [normal3D, norm_lateral];
        
        % add new face within the triangle (with cut)
        new_point_index = [point_i, num_s+index_bottom];
        point_new = [point_as, point_lateral(:,index_bottom)];
        if size(unique(point_new(1,:)),2)==1 % if x is the same for all case, use y and z coordinate to mesh/else use x and z coodinate for mesh
            face_new_orig = delaunay(point_new(2,:),point_new(3,:));
        else
            face_new_orig = delaunay(point_new(1,:),point_new(3,:));
        end
        face_new = [];
        for k = 1:size(face_new_orig,1)
            if min(face_new_orig(k,:))>3
            else
                face_new = [face_new ; face_new_orig(k,:)];
            end            
        end
        for k = 1:size(face_new,1)
            face_new(k,:) = new_point_index(face_new(k,:));            
        end 
        norm_new = b*ones(1,size(face_new,1));
        face3D = [face3D ; face_new];
        normal3D = [normal3D, norm_new];       
    end
    
    % delete repeated face and normal
    index_total = 1:1:size(face3D,1);
    index_del = num_f;
    index_left = setdiff(index_total, index_del);
    face3D = face3D(index_left,:);
    normal3D = normal3D(:,index_left);
        
    figure()
    scatter3(point3D(1,:),point3D(2,:),point3D(3,:),'b');
    hold on;
    for j = 1:size(face3D,1)
        id = zeros(1,4);
        id(1,1:3) = face3D(j,:);
        id(1,4) = face3D(j,1);
        point_id = point3D(:,id);
        fill3(point_id(1,:),point_id(2,:),point_id(3,:),'r');
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

function R = rotate_3D(a,b,FLAG)
R = 2*(a+b)*(a+b)'/((a+b)'*(a+b))-eye(3);
end

function z_max = dist_pl(line, in)
z = zeros(1,size(in,2));
for i = 1:size(in,2)
    A = line(2,2) - line(2,1);
    B = line(1,1) - line(1,2);
    C = -line(1,1)*line(2,2) + line(1,2)*line(2,1);
    z(1,i) = abs(A*in(1,i)+B*in(2,i)+C) / sqrt(A^2+B^2);
end
z_max = min(z);
end

function [c, r] = center_cal(point)
% calculate the center of triangle and the maximum radius
c = mean(point,2); % calculate the center of triangle
% calculate the maximum radius

point = [point, point(:,1)];
R = zeros(1,3);
for i = 1:3
    dir1 = point(:,i)-point(:,i+1);
    dir1 = dir1/norm(dir1);
    dir2 = c-point(:,i);
    R(1,i) = norm(cross(dir1, dir2));
end
r = min(R);
end

function [norm, point, face] = extrude_3D(r_max, z_max)
r = (rand(1)*0.5+0.5)*r_max;
z = (rand(1)*0.5+0.5)*z_max;
% z: extrusion in z axis
cut_shape = randi([1,10],1);       
% point creation
if cut_shape == 1 % circle
    [point_new] = circ(r);
elseif cut_shape == 2 % ellipse
    [point_new] = elli(r); 
else
    [point_new] = poly(r);
end
[norm_new] = norm_cal(point_new, 1); % calculate the norm using exterior
n = randi([1,3],1);
switch n
    case 1 % parallel
        % point
        point = zeros(3,2*size(point_new,2)+1);
        point(1:2,1:size(point_new,2)) = point_new;
        point(1:2,size(point_new,2)+1:2*size(point_new,2)) = point_new;
        point(3,size(point_new,2)+1:2*size(point_new,2)) = z * ones(1,size(point_new,2));
        point(:,end) = [0 ; 0; z];
        % face
        face = zeros(3*size(point_new,2),3);
        norm = zeros(3,3*size(point_new,2));
        point_list1 = zeros(1,size(point_new,2)+1);
        point_list2 = zeros(1,size(point_new,2)+1);
        point_list1(1,1:size(point_new,2)) = 1:1:size(point_new,2);
        point_list1(1,end) = point_list1(1,1);
        point_list2 = point_list1 + size(point_new,2);
        num = 1;
        for i = 1:size(point_new,2)
            % lateral surface and normal
            face(num,:) = [point_list1(1,i), point_list1(1,i+1), point_list2(1,i)]; 
            face(num+1,:) = [point_list2(1,i), point_list2(1,i+1), point_list1(1,i+1)];
            norm(1:2,num:num+1) = norm_new(:,i) * [1 1];
            % top surface and normal
            face(num+2,:) = [point_list2(1,i), point_list2(1,i+1), size(point,2)];
            norm(3,num+2) = 1;
            num = num + 3;
        end
    case 2 % parallel but smaller corss section
        % point
        point = zeros(3,2*size(point_new,2)+1);
        point(1:2,1:size(point_new,2)) = point_new;
        ratio = rand(1)*0.5+0.25; % range from 0.25 to 0.75
        point(1:2,size(point_new,2)+1:2*size(point_new,2)) = ratio*point_new;
        point(3,size(point_new,2)+1:2*size(point_new,2)) = z * ones(1,size(point_new,2));
        point(:,end) = [0 ; 0; z];
        % face
        face = zeros(3*size(point_new,2),3);
        norm = zeros(3,3*size(point_new,2));
        point_list1 = zeros(1,size(point_new,2)+1);
        point_list2 = zeros(1,size(point_new,2)+1);
        point_list1(1,1:size(point_new,2)) = 1:1:size(point_new,2);
        point_list1(1,end) = point_list1(1,1);
        point_list2 = point_list1 + size(point_new,2);
        num = 1;
        for i = 1:size(point_new,2)
            % lateral surface and normal
            face(num,:) = [point_list1(1,i), point_list1(1,i+1), point_list2(1,i)]; 
            face(num+1,:) = [point_list2(1,i), point_list1(1,i+1), point_list2(1,i+1)];
            point_cood = point(:,face(num,:));
            norm(:,num) = norm_triangle(point_cood); %the surface is oblique
            norm(:,num+1) = norm(:,num);
            % top surface and normal
            face(num+2,:) = [point_list2(1,i), point_list2(1,i+1), size(point,2)];
            norm(3,num+2) = 1;
            num = num + 3;
        end
    case 3 % converge to a point
        % point
        point = zeros(3,size(point_new,2)+1);
        point(1:2,1:size(point_new,2)) = point_new;
        point(:,end) = [0 ; 0; z];
        % face
        face = zeros(size(point_new,2),3);
        norm = zeros(3,size(point_new,2));
        point_list1 = zeros(1,size(point_new,2)+1);
        point_list1(1,1:size(point_new,2)) = 1:1:size(point_new,2);
        point_list1(1,end) = point_list1(1,1);
        for i = 1:size(point_new,2)
            % lateral surface and normal
            face(i,:) = [point_list1(1,i), point_list1(1,i+1), size(point,2)]; 
            point_cood = point(:,face(i,:));
            norm(:,i) = norm_triangle(point_cood); %the surface is oblique
        end    
end
end

function [norm_new] = norm_triangle(point)
% calculate normal vector from point coodinate
% obtain point coodinate
p1 = point(:,1);
p2 = point(:,2);
p3 = point(:,3);

% obtain two vectors
dir1 = p2 - p1;
dir2 = p3 - p1;
norm_new = cross(dir1, dir2) / norm(cross(dir1, dir2));
end

function [POINT] = poly(r)
% create polygon with 3 to 10 vertices
% Cham: 1--no chamfer 2--chamfer

num_v = randi([3,10],1);
point = zeros(2,num_v+1);

num_st = randi([0,1],1);
if num_st == 0 % polygon with different angle and side length
    len = r * 0.5*(rand(1,num_v)+1);
    theta = 0:2*pi/num_v:2*pi*(num_v-1)/num_v;
    theta = theta + 2*pi/num_v*(rand(1,num_v)*0.8);  % add random angle to the polygon
else % standard polygon
    len = r * 0.5*(rand(1)+1) * ones(1,num_v);
    theta = 0:2*pi/num_v:2*pi*(num_v-1)/num_v;
end


for j = 1:num_v
    point(1,j) = len(1,j) * cos(theta(1,j));
    point(2,j) = len(1,j) * sin(theta(1,j));    
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
cham = randi([1,2],1);
for j = 1:num_v
    if cham == 1
        POINT = [POINT, Point(:,3*j-2:3*j)];
    else
        POINT_new = Cham(Point(:,3*j-2:3*j));
        POINT = [POINT, POINT_new];        
    end  
end
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

function [point] = circ(r)
% create circle based on r
num = 36; % number of points in a circle is 36 / sample every 10 degree
for i = 1:num
   ang = 10*i*(pi/180);
   point(1,i)= (r*cos(ang));
   point(2,i)= (r*sin(ang));
end

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

function [point] = elli(r)
% create ellipse based on r
a = 0.5*(rand(1)+1)*r;
b = 0.5*(rand(1)+1)*r; 
num = 36; % number of points in a circle is 36 / sample every 10 degree
for i = 1:num
   ang = 10*i*(pi/180);
   point(1,i)= (a*cos(ang));
   point(2,i)= (b*sin(ang));
end

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
% FLAG: 1--exterior 0--interior
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

% clockwise change direction
tf = ispolycw(point(:,1), point(:,2)); % check if the points are listed clockwise
if tf == 1
    normal = -normal; 
end

% interior change direction
if FLAG == 0
    normal = -normal;
end
end
