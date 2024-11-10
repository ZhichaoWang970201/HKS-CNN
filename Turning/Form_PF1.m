function [Point, Face, Normal] = Form_PF1(point)
    len = size(point,1);
    theta_interval = 10; % create a new point every 10 degree
    num_circle = 360 / theta_interval;
    Point = zeros(len*num_circle,3);
    Face = zeros(len*(2*num_circle),3);
    Normal = zeros(size(Face,1),3);
    
    % from the second point to len-1 point 
    for i = 1:len
        Point(1+(i-1)*num_circle:i*num_circle,:) = Form_P(point(i,:),num_circle);     
    end
    
    num_face = 1;
    for i = 1:len-1
        for j = 1:num_circle
            Face(num_face,:) = [(i-1)*num_circle+j i*num_circle+j (i-1)*num_circle+j+1];
            Face(num_face+1,:) = [(i-1)*num_circle+j+1 i*num_circle+j i*num_circle+j+1];
            if j == num_circle
                Face(num_face,:) = [i*num_circle (i+1)*num_circle (i-1)*num_circle+1];
                Face(num_face+1,:) = [(i-1)*num_circle+1 (i+1)*num_circle i*num_circle+1];
            end
            num_face = num_face + 2;
        end
    end
    % the last one
    for j = 1:num_circle
        Face(num_face,:) = [(len-1)*num_circle+j j (len-1)*num_circle+j+1];
        Face(num_face+1,:) = [(len-1)*num_circle+j+1 j j+1];
        if j == num_circle
            Face(num_face,:) = [len*num_circle num_circle (len-1)*num_circle+1];
            Face(num_face+1,:) = [(len-1)*num_circle+1 num_circle 1];
        end
        num_face = num_face + 2;
    end
    
    % create normal
    for i = 1:size(Face,1)
        Point1 = Point(Face(i,1),:);
        Point2 = Point(Face(i,2),:);
        Point3 = Point(Face(i,3),:);
        Normal(i,:) = cross(Point2-Point1, Point1-Point3);
        Normal(i,:) = Normal(i,:) / norm(Normal(i,:));
    end
end


function POINT = Form_P(point, num_circle)
    POINT = zeros(num_circle,3);
    for i = 1:num_circle
        theta = i/num_circle*(2*pi);
        POINT(i,1) = point(1,1);
        POINT(i,2) = point(1,2) * cos(theta);
        POINT(i,3) = point(1,2) * sin(theta);
    end
end
