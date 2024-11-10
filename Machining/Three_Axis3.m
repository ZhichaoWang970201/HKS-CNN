clear all;
close all;
clc;

num_example = 1000; % the number of example to be created at once

% define three point relationship between points
% curve up and curve down is modeled using ellipse curve
% create an index for each operation: horizontal right: 1; vertical up: 2;
% vertical down: 3; line right up: 4; line right down: 5;
% curve right up: 6; curve right down: 7

% Case 1: only the outer is turned
min_point = 3; % the minimum number of point within a x-y plane curve
max_point = 13; % the maximum number of point within a x-y plane curve
x_avg = 0.5; % a reference ratio for increasing in x and y coordinate
for i = 1001:2000
    i
    % the first point is origin: no need to pay attention to that
    point = [];
    point(1,:) = [0  0];
    Point = point(end,:); % index point
    % the second point can only be 'vertical up' or 'line up' or 'curve up'
    %j = randi(3); 
    j = randsrc(1,1, [1,2,3; 1/2,1/3,1/6]); % less probability to create curve
    if j == 1 % vertical up
        point_new = VU(Point, x_avg);
        FLAG = 2; % recording the previous operation is "vertical up"
    elseif j == 2 % line right up
        point_new = LRU(Point, x_avg);
        FLAG = 4; % recording the previous operation is "line right up"
    else % curve up
        point_new = CRU(Point, x_avg);
        FLAG = 6; % recording the previous operation is "curve right up"
    end
    point = [point ; point_new]; % concate the new point with the old ones
    Point = point(end,:); % index point: update
    clear point_new;
    
    % from the third point to last two point can be any one of six
    num = randi(max_point-min_point)+min_point-2; % random creation number of points
    for k = 1:num
        len = length(point);
        %j = randi(7); % any one of the seven choices of function
        j = randsrc(1,1, [1,2,3,4,5,6,7; 2/3,1/12,1/12,1/18,1/18,1/36,1/36]); % less probability to create curve
        while (FLAG==j || (FLAG==2&&j==3) || (FLAG==3&&j==2)) % get rid of two repeated function/get rid of vertical up and vertical down sequentially
            j = randi(7);
        end
        if j == 1
            point_new = HR(Point, x_avg);
            FLAG = 1;
        elseif j == 2
            point_new = VU(Point, x_avg);
            FLAG = 2;
        elseif j == 3
            point_new = VD(Point, x_avg);
            FLAG = 3;
        elseif j == 4
            point_new = LRU(Point, x_avg);
            FLAG = 4;
        elseif j == 5
            point_new = LRD(Point, x_avg);
            FLAG = 5;
        elseif j == 6
            point_new = CRU(Point, x_avg);
            FLAG = 6;
        else
            point_new = CRD(Point, x_avg);
            FLAG = 7;
        end
        point = [point ; point_new];
        Point = point(end,:); % index point
        clear point_new
    end
    
    % the last point can only be 'vertical down' or 'line down' or 'curve down'
    len = length(point);
        
    if FLAG == 2 || FLAG == 3 % if the previsous one is vertical up or down, the final one cannot be vertical down
        %j = randi(2)+1;
        j = randsrc(1,1, [1,2 ; 2/3,1/3])+1;
    else
        %j = randi(3);
        j = randsrc(1,1, [1,2,3 ; 1/2,1/3,1/6]);
    end

    if j == 1 % vertical down
        point(len+1,1) = point(len,1);
        point(len+1,2) = 0;
    elseif j == 2 % line down
        point_new = LRD(Point, x_avg);
        point_new(1,2) = 0;
        point = [point ; point_new];
    else % curve down
        point_new = CRD(Point, x_avg);
        point_new(end,2) = 0;
        point = [point ; point_new];
    end
    clear Point;
    
    z_max = max(point(:,2));
    z_m = 2^(rand(1)*4-3)*z_max;
    point(:,2) = point(:,2)+z_m;
    x_cood = flip(unique(point(:,1)));
    point_new = [x_cood , zeros(size(x_cood,1),1)];
    point = [point ; point_new]; % to form an enclosed circle
    
    ratio_xz = 0.3*rand(1)+0.1; % the length in x axis is 2 to 10 of the height in z axis
    point(:,1) = point(:,1) * (max(point(:,2))/max(point(:,1))) / ratio_xz;      
        
    pgon = polyshape(point(:,1),point(:,2),'SolidBoundaryOrientation','ccw');
    T = triangulation(pgon);
    V = T.Points;
    F = T.ConnectivityList;
    [N] = norm_cal(V', 1);
    
    % extrusion in y axis
    y_dist = 2^(2*rand(1)-1)*max(V(:,1)); % x / y plane ratio is 0.5 to 2
    point3D = [];
    face3D = [];
    normal3D = [];
    
    % front plane
    point_new = zeros(3,size(V,1));
    point_new([1,3],:) = V';
    normal_new = zeros(3,size(F,1));
    normal_new(2,:) = -ones(1,size(F,1));
    point3D = [point3D, point_new];
    normal3D = [normal3D, normal_new];
    face3D = [face3D; F];
    clear point_new normal_new;
    
    % behind plane
    point_new = zeros(3,size(V,1));
    point_new([1,3],:) = V';
    point_new(2,:) = y_dist*ones(1,size(V,1));
    normal_new = zeros(3,size(F,1));
    normal_new(2,:) = ones(1,size(F,1));
    point3D = [point3D, point_new];
    normal3D = [normal3D, normal_new];
    face3D = [face3D; F+size(V,1)];
    clear point_new normal_new;
    clear T_new;
    
    % lateral surface
    point_list1 = [1:1:size(V,1),1]; % front index
    point_list2 = point_list1+size(V,1); % behind index
        
    for k = 1:size(V,1)
        face_new  = [point_list1(1,k) point_list1(1,k+1) point_list2(1,k) ; point_list1(1,k+1) point_list2(1,k) point_list2(1,k+1)];
        normal_new = [N(1,k); 0; N(2,k)]*ones(1,2);
        face3D = [face3D; face_new];
        normal3D = [normal3D, normal_new];
    end
        
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

function Point_new = HR(Point, x_avg)
    lamda = 2^(rand(1)*2-1);
    Point_new(:,1) = Point(:,1) + lamda * x_avg;
    Point_new(:,2) = Point(:,2);
end

function Point_new = VU(Point, x_avg)
    lamda = 2^(rand(1)*2-1);
    Point_new(:,1) = Point(:,1);
    Point_new(:,2) = Point(:,2) + lamda * x_avg;
end

function Point_new = VD(Point, x_avg)
    lamda = -2^(rand(1)*2-1);
    Point_new(:,1) = Point(:,1);
    Point_new(:,2) = max(Point(:,2) + lamda * x_avg, Point(:,2)/2); % set lower bound
end

function Point_new = LRU(Point, x_avg)  
    lamda1 = 2^(rand(1)*2-1);
    lamda2 = 2^(rand(1)*2-1);
    Point_new(:,1) = Point(:,1) + lamda1 * x_avg;
    Point_new(:,2) = Point(:,2) + lamda2 * x_avg;
end

function Point_new = LRD(Point, x_avg)
    lamda1 = 2^(rand(1)*2-1);
    lamda2 = -2^(rand(1)*2-1);
    Point_new(:,1) = Point(:,1) + lamda1 * x_avg;
    Point_new(:,2) = max(Point(:,2) + lamda2 * x_avg, Point(:,2)/2); % set lower bound
end

function Point_new = CRU(Point, x_avg) % model it using 1/4 of an ellipse
    lamda1 = 2^(rand(1)*2-1);
    lamda2 = 2^(rand(1)*2-1);
    a = lamda1 * x_avg;
    b = lamda2 * x_avg;
    index = randi([1,2],1); % index 1: second quadrant / index 2: fourth quadrant
    num_point = 9;
    theta_incr = 10;
    Point_new = zeros(num_point,2);
    
    if index == 1
        % create points in the second quadrant
        C = Point +  [a, 0];
        for i = 1:num_point
            theta = (180-theta_incr*i)/180*pi;
            Point_new(i,:) = C + [a*cos(theta), b*sin(theta)];
        end
    else
        % create points in the fourth quadrant
        C = Point + [0, b];
        for i = 1:num_point
            theta = (270+theta_incr*i)/180*pi;
            Point_new(i,:) = C + [a*cos(theta), b*sin(theta)];
        end
    end  
end
 
% a little modification of CRU
function Point_new = CRD(Point, x_avg)
    lamda1 = 2^(rand(1)*2-1);
    lamda2 = 2^(rand(1)*2-1);
    a = lamda1 * x_avg;
    b = min(lamda2 * x_avg, Point(1,2)/2);
    index = randi([1,2],1); % index 1: second quadrant / index 2: fourth quadrant
    num_point = 9;
    theta_incr = 10;
    Point_new = zeros(num_point,2);
    
    if index == 1
        % create points in the third quadrant
        C = Point +  [a, 0];
        for i = 1:num_point
            theta = (180+theta_incr*i)/180*pi;
            Point_new(i,:) = C + [a*cos(theta), b*sin(theta)];
        end
    else
        % create points in the first quadrant
        C = Point + [0, -b];
        for i = 1:num_point
            theta = (90-theta_incr*i)/180*pi;
            Point_new(i,:) = C + [a*cos(theta), b*sin(theta)];
        end
    end  
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