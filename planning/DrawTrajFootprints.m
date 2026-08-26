function DrawTrajFootprints(x,y,theta)
    global params_
    for ii=1:length(x)
        V=CreateVehiclePolygon(x(ii),y(ii),theta(ii),2);
        plot(V.x,V.y,'Color',params_.utility.ego_vehicle_rgb);
    end
end