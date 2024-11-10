function write_txt(Point,Face,Normal,name)
    fileID = fopen(name,'w');
    
    fprintf(fileID,'Point\n');
    for i = 1:size(Point,1)
        fprintf(fileID,'%3.6f, %3.6f, %3.6f\n', Point(i,1), Point(i,2), Point(i,3)); 
    end
    
    fprintf(fileID,'\n');
    fprintf(fileID,'Face\n');
    for i = 1:size(Face,1)
        fprintf(fileID,'%d, %d, %d\n', Face(i,1), Face(i,2), Face(i,3)); 
    end
    
    fprintf(fileID,'\n');
    fprintf(fileID,'Normal\n');
    for i = 1:size(Normal,1)
        fprintf(fileID,'%3.6f, %3.6f, %3.6f\n', Normal(i,1), Normal(i,2), Normal(i,3)); 
    end
    
    fclose(fileID);

end