clear all;
close all;
clc;

num_example = 500; % the number of example to be created at once

% define three point relationship between points
% curve up and curve down is modeled using sin curve
% create an index for each operation: horizontal right: 1; vertical up: 2;
% vertical down: 3; line right up: 4; line right down: 5;
% curve right up: 6; curve right down: 7
relationship = {'horizontal_right' ; 'vertical_up' ; 'vertical_down' ; ... 
'line_right_up' ; 'line_right_down' ; 'curve_right_up' ; 'curve_right_down'};

% Case 1: only the outer is turned
min_point = 1; % the minimum number of point within a x-y plane curve
max_point = 5; % the maximum number of point within a x-y plane curve
min_scale = 1; % the minimum scale between x axis and y axis
max_scale = 8; % the maximum scale between x axis and y axis
x_avg = 0.5; % a reference ratio for increasing in x and y coordinate
for i = 1:num_example
    tic % start recording time used
    i
    % the first point is origin: no need to pay attention to that
    point(1,:) = [0  0];
    Point = point(end,:); % index point
    % the second point can only be 'vertical up' or 'line up' or 'curve up'
    j = randi(3); 
    if j == 1 % vertical up
        point_new = VU(Point, x_avg);
        FLAG = 2; % recording the previous operation is "vertical up"
    elseif j == 2 % line right up
        point_new = LRU(Point, x_avg);
        FLAG = 4; % recording the previous operation is "line right up"
    else % curve up
        point_new = CRU(Point, x_avg);
        y_max = max(point_new(:,2));
        for jj = 1:size(point_new,1) % model the curve as sine, can have very small value or even negative (get rid of that)
            if point_new(jj,2) < y_max/2
                point_new(jj,2) = y_max/2;
            end
        end
        FLAG = 6; % recording the previous operation is "curve right up"
    end
    point = [point ; point_new]; % concate the new point with the old ones
    Point = point(end,:); % index point: update
    clear point_new;
    
    % from the third point to last two point can be any one of six
    num = randi(max_point-min_point)+min_point-1; % random creation number of points
    for k = 1:num
        len = length(point);
        j = randi(7); % any one of the seven choices of function
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
        j = randi(2)+1;
    else
        j = randi(3);    
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
    
    % Case1: only turn exterior surface
    point = ReScale(point,min_scale,max_scale); % scale the ratio of x and y directions
    [Point, Face, Normal] = Form_PF(point);
    
    figure()
    plot3(Point(:,1), Point(:,2), Point(:,3), 'b*');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    hold on
    for j = 1:size(Face,1)      
%         if j > size(Face,1)-72 || j < 72
            point_plot = [Point(Face(j,1),:);Point(Face(j,2),:);Point(Face(j,3),:);Point(Face(j,1),:)];
            plot3(point_plot(:,1), point_plot(:,2), point_plot(:,3), 'r'); 
            hold on;
%             point_start = mean(point_plot(1:3,:));
%             point_end = point_start + Normal(j,:)/2;
%             point_normal = [point_start ; point_end];
%             plot3(point_normal(:,1), point_normal(:,2), point_normal(:,3), 'k');
%             hold on;
%             plot3(point_start(:,1), point_start(:,2), point_start(:,3), 'gx');
%             hold on;
%             plot3(point_end(:,1), point_end(:,2), point_end(:,3), 'm*');
%         end
    end
    
    name = strcat('case1_',num2str(i),'.fig');
    savefig(name);
    figure()
    plot(point(:,1),point(:,2),'b*');
    hold on
    plot(point(:,1),point(:,2),'r');
    name = strcat('case1_',num2str(i),'_single.fig');
    savefig(name);
        
    close all;
    
    name = strcat('case1_',num2str(i),'.txt');
    write_txt(Point,Face,Normal,name);
    
    % Case 2: both the outer and inter is turned
    point2 = point;
    y_max = rand(1)*max(point2(:,2));
    x_max = max(point2(:,1));
    j = randi(3);
    if j == 1
        point2(:,2) = point2(:,2) + y_max; % move points up in y direction
    elseif j == 2 % three lines connecting two ends
        point2(:,2) = point2(:,2) + y_max;
        point_new2(1,:) = [point2(end,1)-rand(1)*x_max , point2(end,2)-rand(1)*y_max];
        point_new2(2,:) = [rand(1)*point_new2(1,1) , rand(1)*y_max];
        point2 = [point2 ; point_new2];        
    else % two arcs and one line connecting two ends
        point2(:,2) = point2(:,2) + y_max;
        x1 = 0.5*rand(1)*x_max;
        x2 = 0.5*rand(1)*x_max;
        y1 = rand(1)*y_max;
        y2 = rand(1)*y_max;
        for k = 1:9 % 90/10 the first one is omitted since it is the last point of "point"
            theta = (k*10) / 180 * pi; %transfer back to theta
            point_new1(k,1) = point2(end,1) - x1/9*k;
            point_new1(k,2) = point2(end,2) - y1 * sin(theta);
        end
        for k = 0:8 % the last one is omitted because it is the original point
            theta = ((9-k)*10) / 180 * pi;
            point_new2(k+1,1) = (9-k)/9*x2;
            point_new2(k+1,2) = y_max - (y_max-y2)*sin(theta);
        end      
        point2 = [point2 ; point_new1 ; point_new2];
    end
    
    point2 = ReScale(point2,min_scale,max_scale);
    [Point, Face, Normal] = Form_PF1(point2);
    
    figure()
    plot3(Point(:,1), Point(:,2), Point(:,3), 'b*');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    hold on
    for j = 1:size(Face,1)      
%         if j > size(Face,1)-108 || j < 108
            point_plot = [Point(Face(j,1),:);Point(Face(j,2),:);Point(Face(j,3),:);Point(Face(j,1),:)];
            plot3(point_plot(:,1), point_plot(:,2), point_plot(:,3), 'r'); 
            hold on;
%             point_start = mean(point_plot(1:3,:));
%             point_end = point_start + Normal(j,:)/2;
%             point_normal = [point_start ; point_end];
%             plot3(point_normal(:,1), point_normal(:,2), point_normal(:,3), 'k');
%             hold on;
%             plot3(point_start(:,1), point_start(:,2), point_start(:,3), 'gx');
%             hold on;
%             plot3(point_end(:,1), point_end(:,2), point_end(:,3), 'm*');
%         end
    end
    
    name = strcat('case2_',num2str(i),'.fig');
    savefig(name);
    figure()
    plot(point2(:,1),point2(:,2),'b*');
    hold on
    plot(point2(:,1),point2(:,2),'r');
    name = strcat('case2_',num2str(i),'_single.fig');
    savefig(name);
    
    close all;
    
    name = strcat('case2_',num2str(i),'.txt');
    write_txt(Point,Face,Normal,name);
    
    % Case 3: both the outer and left inter is turned
    point3 = point;
    y_max = rand(1)*max(point3(:,2));
    x_max = max(point3(:,1));
    j = randi(2);
    if j == 1
        point3(:,2) = point3(:,2) + y_max; % move points up in y direction
        point_new = [ rand(1)*x_max 0 ; rand(1)*x_max y_max];
        point3 = [point_new ; point3];
    else % three lines connecting two ends
        point3(:,2) = point3(:,2) + y_max;
        point_new = [ rand(1)*x_max 0 ; rand(1)*x_max rand(1)*y_max];
        point3 = [point_new ; point3];        
    end
    point3(end,2) = 0;
    
    point3 = ReScale(point3,min_scale,max_scale);
    [Point, Face, Normal] = Form_PF(point3);
    
    figure()
    plot3(Point(:,1), Point(:,2), Point(:,3), 'b*');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    hold on
    for j = 1:size(Face,1)      
%         if j > size(Face,1)-108 || j < 108
            point_plot = [Point(Face(j,1),:);Point(Face(j,2),:);Point(Face(j,3),:);Point(Face(j,1),:)];
            plot3(point_plot(:,1), point_plot(:,2), point_plot(:,3), 'r'); 
            hold on;
%             point_start = mean(point_plot(1:3,:));
%             point_end = point_start + Normal(j,:)/10;
%             point_normal = [point_start ; point_end];
%             plot3(point_normal(:,1), point_normal(:,2), point_normal(:,3), 'k');
%             hold on;
%             plot3(point_start(:,1), point_start(:,2), point_start(:,3), 'gx');
%             hold on;
%             plot3(point_end(:,1), point_end(:,2), point_end(:,3), 'm*');
%         end
    end
    
    name = strcat('case3_',num2str(i),'.fig');
    savefig(name);
    figure()
    plot(point3(:,1),point3(:,2),'b*');
    hold on
    plot(point3(:,1),point3(:,2),'r');
    name = strcat('case3_',num2str(i),'_single.fig');
    savefig(name);
    close all;
    
    name = strcat('case3_',num2str(i),'.txt');
    write_txt(Point,Face,Normal,name);
    
    % Case 4: both the outer and right inter is turned
    point4 = point;
    y_max = rand(1)*max(point4(:,2));
    x_max = max(point4(:,1));
    j = randi(2);
    if j == 1
        point4(:,2) = point4(:,2) + y_max; % move points up in y direction
        point_new = [ rand(1)*x_max y_max ; rand(1)*x_max 0 ];
        point4 = [point4 ; point_new];
    else % three lines connecting two ends
        point4(:,2) = point4(:,2) + y_max;
        point_new = [ rand(1)*x_max rand(1)*y_max ; rand(1)*x_max 0 ];
        point4 = [point4 ; point_new];        
    end
    point4(1,2) = 0;
    
    point4 = ReScale(point4,min_scale,max_scale);
    [Point, Face, Normal] = Form_PF(point4);
    
    figure()
    plot3(Point(:,1), Point(:,2), Point(:,3), 'b*');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    hold on
    for j = 1:size(Face,1)      
%         if j > size(Face,1)-108 || j < 108
            point_plot = [Point(Face(j,1),:);Point(Face(j,2),:);Point(Face(j,3),:);Point(Face(j,1),:)];
            plot3(point_plot(:,1), point_plot(:,2), point_plot(:,3), 'r'); 
            hold on;
%             point_start = mean(point_plot(1:3,:));
%             point_end = point_start + Normal(j,:)/10;
%             point_normal = [point_start ; point_end];
%             plot3(point_normal(:,1), point_normal(:,2), point_normal(:,3), 'k');
%             hold on;
%             plot3(point_start(:,1), point_start(:,2), point_start(:,3), 'gx');
%             hold on;
%             plot3(point_end(:,1), point_end(:,2), point_end(:,3), 'm*');
%         end
    end
    
    name = strcat('case4_',num2str(i),'.fig');
    savefig(name);
    figure
    plot(point4(:,1),point4(:,2),'b*');
    hold on
    plot(point4(:,1),point4(:,2),'r');
    name = strcat('case4_',num2str(i),'_single.fig');
    savefig(name);
    close all;
    
    name = strcat('case4_',num2str(i),'.txt');
    write_txt(Point,Face,Normal,name);
    
    
    clear i j FLAG k len num num1 point Point1 Point2 point_new;
    toc;
end

function Point = ReScale(point,min_scale,max_scale)
    scale = rand(1)*(max_scale-min_scale)+min_scale;
    Point = point;
    Point(:,1) = Point(:,1)/max(Point(:,1));
    Point(:,1) = scale * Point(:,1);
    Point(:,2) = Point(:,2)/max(Point(:,2));
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

function Point_new = CRU(Point, x_avg)
    lamda1 = 2^(rand(1)*2);
    lamda2 = 2^(rand(1)*2-1);
    delta_x = lamda1 * x_avg;
    delta_y = lamda2 * x_avg;
    theta_max = rand(1) * pi;
    theta_min = (rand(1) - 1) * pi;
    delta_theta = theta_max - theta_min;
    delta_theta_threshold = pi / 2; 
    if delta_theta < delta_theta_threshold
        theta_max = theta_max + delta_theta_threshold/2;
        theta_min = theta_min - delta_theta_threshold/2;
        delta_theta = delta_theta + delta_theta_threshold;
    end
    A = [sin(theta_min) 1 ; sin(theta_max) 1];
    B = [0 ; delta_y];
    C = A\B;
    a = C(1,1);
    b = C(2,1);
    num_point = floor(delta_theta / 2 / pi * 360 / 10); % sample every 10 degree 
    Point_new = zeros(num_point+1,2);
    for i = 1:num_point
        Point_new(i,1) = Point(1,1) + delta_x / delta_theta * (pi/180) * 10*i;
        Point_new(i,2) = max(Point(1,2) + a * sin(theta_min + (pi/180) * 10*i) + b, Point(1,2)/2);
    end
    Point_new(end,1) = Point(1,1) + delta_x;
    Point_new(end,2) = max(Point(1,2) + delta_y, Point(1,2)/2);
end
 
% a little modification of CRU
function Point_new = CRD(Point, x_avg)
    lamda1 = 2^(rand(1)*2);
    lamda2 = -2^(rand(1)*2-1);
    delta_x = lamda1 * x_avg;
    delta_y = lamda2 * x_avg;
    theta_max = (rand(1) + 1) * pi;
    theta_min = rand(1) * pi;
    delta_theta = theta_max - theta_min;
    delta_theta_threshold = pi / 2; 
    if delta_theta < delta_theta_threshold
        theta_max = theta_max + delta_theta_threshold/2;
        theta_min = theta_min - delta_theta_threshold/2;
        delta_theta = delta_theta + delta_theta_threshold;
    end
    A = [sin(theta_min) 1 ; sin(theta_max) 1];
    B = [0 ; delta_y];
    C = A\B;
    a = C(1,1);
    b = C(2,1);
    num_point = floor(delta_theta / 2 / pi * 360 / 10); 
    Point_new = zeros(num_point+1,2);
    for i = 1:num_point
        Point_new(i,1) = Point(1,1) + delta_x / delta_theta * (pi/180) * 10*i;
        Point_new(i,2) = max(Point(1,2) + a * sin(theta_min + (pi/180) * 10*i) + b, Point(1,2)/2); % set lower bound
    end
    Point_new(end,1) = Point(1,1) + delta_x;
    Point_new(end,2) = max(Point(1,2) + delta_y, Point(1,2)/2);
end