function cw1_dr_kalman()

    gnss = csvread("ans/CW_GNSS_Pos_Vel.csv");
    car_dr = csvread("data/Lawnmower_DR.csv");
    
    deg_to_rad = 0.01745329252; % Degrees to radians conversion factor
    rad_to_deg = 1/deg_to_rad; % Radians to degrees conversion factor
    
    epoches = size(gnss, 1);
    
    t = 0.5; % time interval
    u_r = 10; % initial position uncertainty
    u_v = 0.1; % initial velocity uncertainty
    e_gr = 10; % GNSS position std
    e_gv = 0.02; % velocity std
    S_DR = 0.2; % DR velocity error power spectral density
    
    dr_solution = zeros(epoches, 6);
    
    lat_G = gnss(1, 2) * deg_to_rad; % initial lattitude
    long_G = gnss(1, 3) * deg_to_rad; % initial longtitude
    h = gnss(1, 4); % initial height
    
    [R_N,R_E] = Radii_of_curvature(lat_G);
    
    % state filter
    x = [0; 0; 0; 0];
    % state estimation error covariance matrix 
    P = [u_v^2 0     0                   0;
         0     u_v^2 0                   0;
         0     0     (u_r/(R_N + h))^2 0;
         0     0     0                   (u_r/((R_E + h)*cos(lat_G)))^2];
    

    for i = 1:epoches
    
        % transition matrix
        T = [1           0                        0 0;
             0           1                        0 0;
             t/(R_N + h) 0                        1 0;
             0           t/((R_E + h)*cos(lat_G)) 0 1];
        
        %  system noise covariance matrix
        q1 = (S_DR * t^2) / (2 * (R_N + h));
        q2 = (S_DR * t^2) / (2 * (R_E + h) * cos(lat_G));
        q3 = (S_DR * t^3) / (3 * (R_N + h)^2);
        q4 = (S_DR * t^3) / (3 * ((R_E + h) * cos(lat_G))^2);
        Q = [S_DR*t 0      q1 0;
             0      S_DR*t 0  q2;
             q1     0      q3 0;
             0      q2     0  q4];
    
        lat_G = gnss(i, 2) * deg_to_rad; 
        long_G = gnss(i, 3) * deg_to_rad; 
        h = gnss(i, 4); 
        v_n_G = gnss(i, 5);
        v_e_G = gnss(i, 6);
    
        lat_D = car_dr(i, 2) * deg_to_rad;
        long_D = car_dr(i, 3) * deg_to_rad;
        v_n_D = car_dr(i, 4);
        v_e_D = car_dr(i, 5);
        
        [R_N,R_E] = Radii_of_curvature(lat_G);
    
        x = T * x;
    
        P = T * P * T.' + Q;
        
        % measurement matrix 
        H = [ 0  0 -1  0;
              0  0  0 -1; 
             -1  0  0  0; 
              0 -1  0  0];
    
        % measurement noise covariance matrix
        R = [(e_gr/(R_N + h))^2 0                             0      0; 
             0                  (e_gr/((R_E + h)*cos(lat_G)))^2 0      0; 
             0                  0                             e_gv^2 0; 
             0                  0                             0      e_gv^2];
    
        K = P * H.' / (H * P * H.' + R);
    
        z = [lat_G - lat_D; long_G - long_D; v_n_G - v_n_D; v_e_G - v_e_D];
        z = z - H * x;
    
        x = x + K * z;
    
        P = (eye(4) - K * H) * P;
    
        dr_solution(i, 1) = gnss(i, 1);
        dr_solution(i, 2) = round((lat_D - x(3)) * rad_to_deg, 6);
        dr_solution(i, 3) = round((long_D - x(4)) * rad_to_deg, 6);
        dr_solution(i, 4) = round(v_n_D - x(1), 3);
        dr_solution(i, 5) = round(v_e_D - x(2), 3);
        dr_solution(i, 6) = round(car_dr(i, 6), 2);
    
    end
    
    filename = 'ans/Corrected_DR.csv';
    writematrix(dr_solution, filename);
end


