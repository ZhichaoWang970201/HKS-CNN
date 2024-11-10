clear all;
close all;
clc;

for i = 2001:3000
    name = strcat(num2str(i-1000),'.mat');
    load(name);
    point_l = point3D;
    face_l = face3D;
    normal_l = normal3D;
    clear point3D face3D normal3D;
    j = randi(200);
    name = strcat('cut',num2str(j),'.mat');
    load(name);
    point_r = point3D;
    point_r(1,:) = point_r(1,:) - min(point_r(1,:));
    point_r(2,:) = point_r(2,:) - min(point_r(2,:));
    face_r = face3D;
    normal_r = normal3D;
    clear point3D face3D normal3D;
    
    % find the right points of part_l
    x_max = max(point_l(1,:));
    index_max = find(x_max == point_l(1,:));
    point_b_l = point_l(:,index_max);
    
    % find the left points of part 2
    x_min = min(point_r(1,:));
    index_min = find(x_min == point_r(1,:));
    point_b_r = point_r(:,index_min);
    
    % obtain the face index to be deleted: part 1
    index_l = [];
    for j = 1:size(face_l,1)
        if all(ismember(face_l(j,:),index_max))
            index_l = [index_l ; j];
        end
    end
    list_1 = setdiff([1:1:size(face_l,1)],index_l);
    face_l = face_l(list_1,:);
    normal_l = normal_l(:,list_1);
 
    % obtain the face index to be deleted: part 2
    index_r = [];
    for j = 1:size(face_r,1)
        if all(ismember(face_r(j,:),index_min))
            index_r = [index_r ; j];
        end
    end
    list_r = setdiff([1:1:size(face_r,1)],index_r);
    face_r = face_r(list_r,:);
    normal_r = normal_r(:,list_r);
    
    % the ratio of y and z
    z_ratio = max(point_b_l(3,:)) / max(point_b_r(3,:));
    y_ratio = max(point_l(2,:)) / max(point_r(2,:));
    x_ratio = (rand(1)*0.25+0.25) * max(point_l(1,:)) / max(point_r(1,:));
    
    point_r(1,:) = point_r(1,:)*x_ratio;
    point_r(1,:) = point_r(1,:) - min(point_r(1,:)) + max(point_l(1,:));
    point_r(2,:) = point_r(2,:)*y_ratio;
    point_r(3,:) = point_r(3,:)*z_ratio;
    
    % update normal vector after change the x/y/z ratio
    for j = 1:size(face_r,2)
        point_id = face_r(j,:);
        point = point_r(:,point_id); 
        dir1 = point(:,2)-point(:,1); 
        dir2 = point(:,3)-point(:,1);
        dir = cross(dir1,dir2) / norm(cross(dir1,dir2));
        if dot(dir,normal_r(:,j))<0
            dir = -dir; 
        end
        normal_r(:,j) = dir; 
    end
    
    % concate them together
    point3D = [point_l, point_r];
    face3D = [face_l ; face_r+size(point_l,2)];
    normal3D = [normal_l, normal_r];
    
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
end