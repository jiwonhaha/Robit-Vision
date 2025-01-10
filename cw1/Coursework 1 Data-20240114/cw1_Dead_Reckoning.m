function converted_dr = cw1_Dead_Reckoning(lat, long, h, dead_reckoning)

deg_to_rad = 0.01745329252; % Degrees to radians conversion factor
rad_to_deg = 1/deg_to_rad; % Radians to degrees conversion factor

L = 0.5; % Wheel Base

epoches = size(dead_reckoning, 1);

converted_dr = zeros(epoches, 6);

lat = lat * deg_to_rad;
long = long * deg_to_rad;

for i = 1:epoches

    v_forward = (dead_reckoning(i, 2) + dead_reckoning(i, 3)...
            + dead_reckoning(i, 4) + dead_reckoning(i, 5)) / 4;

    if v_forward == 0
        v_lat = 0;
    else
        delta = (dead_reckoning(i, 6) * L) / v_forward;
        v_lat = v_forward * tan(delta);
    end

    heading = dead_reckoning(i, 7) * deg_to_rad;

    v_n = v_forward * cos(heading) - v_lat * sin(heading);
    v_e = v_forward * sin(heading) + v_lat * cos(heading);

    [R_N,R_E] = Radii_of_curvature(lat);

    lat = lat + (v_n * 0.5 / (R_N + h));
    long = long + (v_e * 0.5 / ((R_E + h) * cos(lat)));

    converted_dr(i, 1) = dead_reckoning(i, 1);
    converted_dr(i, 2) = lat * rad_to_deg;
    converted_dr(i, 3) = long * rad_to_deg;
    converted_dr(i, 4) = v_n;
    converted_dr(i, 5) = v_e;
    converted_dr(i, 6) = dead_reckoning(i, 7);
end
