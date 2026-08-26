function Form4WISInitialGuess(x,y,theta,v,a,phy,w,terminal_time,pattern)
    global params_
    nfe=size(x,2);
    phy(phy>params_.vehicle.phymax)=params_.vehicle.phymax;
    phy(phy<-params_.vehicle.phymax)=-params_.vehicle.phymax;
    v(v>params_.vehicle.vmax)=params_.vehicle.vmax;
    v(v<-params_.vehicle.vmax)=-params_.vehicle.vmax;
    a(a>params_.vehicle.amax)=params_.vehicle.amax;
    a(a<-params_.vehicle.amax)=-params_.vehicle.amax;
    w(w>params_.vehicle.wmax)=params_.vehicle.wmax;
    w(w<-params_.vehicle.wmax)=-params_.vehicle.wmax;
    
    xr=x-params_.vehicle.FWISr2p.*cos(theta);
    yr=y-params_.vehicle.FWISr2p.*sin(theta);
    xf=x+params_.vehicle.FWISf2p.*cos(theta);
    yf=y+params_.vehicle.FWISf2p.*sin(theta);
    
    if pattern==0
        delete('ig_diag.INIVAL');
        fid=fopen('ig_diag.INIVAL','w');
        for ii=1:nfe
            fprintf(fid,'let x[%g] := %f;\r\n',ii,x(ii));
            fprintf(fid,'let y[%g] := %f;\r\n',ii,y(ii));
            fprintf(fid,'let theta[%g] := %f;\r\n',ii,theta(ii));
            fprintf(fid,'let v[%g] := %f;\r\n',ii,v(ii));
            fprintf(fid,'let a[%g] := %f;\r\n',ii,a(ii));
            fprintf(fid,'let phy[%g] := %f;\r\n',ii,phy(ii));
            fprintf(fid,'let w[%g] := %f;\r\n',ii,w(ii));
            fprintf(fid,'let xf[%g] := %f;\r\n',ii,xf(ii));
            fprintf(fid,'let yf[%g] := %f;\r\n',ii,yf(ii));
            fprintf(fid,'let xr[%g] := %f;\r\n',ii,xr(ii));
            fprintf(fid,'let yr[%g] := %f;\r\n',ii,yr(ii));
        end
        fprintf(fid,'let tf := %f;\r\n',terminal_time);
        fclose(fid);
    elseif pattern==2
        delete('ig_crab.INIVAL');
        fid=fopen('ig_crab.INIVAL','w');
        for ii=1:nfe
            fprintf(fid,'let x[%g] := %f;\r\n',ii,x(ii));
            fprintf(fid,'let y[%g] := %f;\r\n',ii,y(ii));
            fprintf(fid,'let theta[%g] := %f;\r\n',ii,theta(ii));
            fprintf(fid,'let v[%g] := %f;\r\n',ii,v(ii));
            fprintf(fid,'let a[%g] := %f;\r\n',ii,a(ii));
            fprintf(fid,'let phy[%g] := %f;\r\n',ii,phy(ii));
            fprintf(fid,'let w[%g] := %f;\r\n',ii,w(ii));
            fprintf(fid,'let xf[%g] := %f;\r\n',ii,xf(ii));
            fprintf(fid,'let yf[%g] := %f;\r\n',ii,yf(ii));
            fprintf(fid,'let xr[%g] := %f;\r\n',ii,xr(ii));
            fprintf(fid,'let yr[%g] := %f;\r\n',ii,yr(ii));
        end
        fprintf(fid,'let tf := %f;\r\n',terminal_time);
        fclose(fid);        
    end
end