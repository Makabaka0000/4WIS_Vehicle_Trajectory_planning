function Write4WISBoundartValues(x,y,theta)
    delete('SixBoundaryValues');
    fid = fopen('SixBoundaryValues', 'w');
    fprintf(fid, '1  %f\r\n', x(1,1));
    fprintf(fid, '2  %f\r\n', y(1,1));
    fprintf(fid, '3  %f\r\n', theta(1,1));
    fprintf(fid, '4  %f\r\n',x(1,end));
    fprintf(fid, '5  %f\r\n', y(1,end));
    fprintf(fid, '6  %f\r\n', theta(1,end));
    fclose(fid);
end