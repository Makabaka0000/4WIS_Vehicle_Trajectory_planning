function asd()
    global params_
    DrawParkingScenario();
    DrawTrajFootprints(params_.limo.x,params_.limo.y,params_.limo.theta);
    plot(params_.ha_result.x,params_.ha_result.y,'r','LineWidth',2);
    plot(params_.limo.x,params_.limo.y,'b','LineWidth',2);
end

function DrawTrajFootprints(x,y,theta)
    global params_
    for ii=1:length(x)
        V=CreateVehiclePolygon(x(ii),y(ii),theta(ii),2);
        plot(V.x,V.y,'Color',params_.utility.ego_vehicle_rgb);
    end
end