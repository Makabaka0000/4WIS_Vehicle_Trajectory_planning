function WritefronCAC(filename)
    global params_
    lamda=params_.corridor_adaptive;
    box_edge_length=zeros(params_.opti.nfe,4);
    max_box=params_.opti.stc.front_stc;
    min_box=params_.stc.min_front;
    for i=1:size(box_edge_length,1)
        left=lamda*abs(min_box(i,1)-max_box(i,1));
        right=lamda*abs(min_box(i,2)-max_box(i,2));
        down=lamda*abs(min_box(i,3)-max_box(i,3));
        up=lamda*abs(min_box(i,4)-max_box(i,4));
        box_edge_length(i,1)=min_box(i,1)-left;
        box_edge_length(i,2)=min_box(i,2)+right;
        box_edge_length(i,3)=min_box(i,3)-down;
        box_edge_length(i,4)=min_box(i,4)+up;
    end
    delete(['STC_',filename]);
    fid=fopen(['STC_',filename],'w');
    for ii=1:length(box_edge_length)
        for jj=1:4
            fprintf(fid,'%g %g %.6f \r\n',ii,jj,box_edge_length(ii,jj));
        end
    end
    fclose(fid);
end