function cw1_kalman()

deg_to_rad = 0.01745329252; % Degrees to radians conversion factor
rad_to_deg = 1/deg_to_rad; % Radians to degrees conversion factor
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
Omega_ie = Skew_symmetric([0,0,omega_ie]);

%initialising state vector x_0 and error covaraicne matrix P_0
[x_0, P_0] = Initialize_Pos;

% Read pseudo-range and pseudo-range rate data
ranges = csvread("data/Pseudo_ranges.csv");
range_rates = csvread("data/Pseudo_range_rates.csv");
epochs = size(ranges, 1) - 1; % Time variable

% Threshold of outlier detection
T = 6;

Pos_vel_NEC = {'Time', 'Latitude (deg)', 'Longitude (deg)', 'Height', 'Velo(N)', 'Velo(E)', 'Velo(D)'};

% Loop through all epochs
for epoch = 2:epochs+1
    
    % time interval
    i = 0.5;
    % Transition matrix (step 1)
    t = [1 0 0 i 0 0 0 0;...
         0 1 0 0 i 0 0 0;...
         0 0 1 0 0 i 0 0;...
         0 0 0 1 0 0 0 0;...
         0 0 0 0 1 0 0 0;...
         0 0 0 0 0 1 0 0;...
         0 0 0 0 0 0 1 i;...
         0 0 0 0 0 0 0 1];
    
    % Acceleration
    S_e_a = 5;
    % Clock phase
    S_a_c = 0.01;
    % Clock frequency
    S_a_cf = 0.04;
    
    % System noise covariance matrix (step 2)
    q1 = S_e_a * i^3 / 3;
    q2 = S_e_a * i^2 / 2;
    q3 = S_e_a * i;
    q4 = (S_a_c * i) + (S_a_cf * i^3 / 3);
    q5 = S_a_cf * i^2 / 2;
    q6 = S_a_cf * i;
    
    Q = [q1  0  0 q2  0  0  0  0;...
          0 q1  0  0 q2  0  0  0;...
          0  0 q1  0  0 q2  0  0;...
         q2  0  0 q3  0  0  0  0;...
          0 q2  0  0 q3  0  0  0;...
          0  0 q2  0  0 q3  0  0;...
          0  0  0  0  0  0 q4 q5;...
          0  0  0  0  0  0 q5 q6;];

    % State estimate (step 3)
    x_1 = t * x_0;
    
    % Upadate error covaricance matrix (step 4)
    P_1 = t * P_0 * t.' + Q;

    %Compute positon and Velocity of Satellite 
    n_sat = size(ranges, 2) - 1;
    sat_r_arr = zeros(n_sat, 3);
    sat_v_arr = zeros(n_sat, 3);
    for i = 1:n_sat
        time = ranges(epoch,1);
        j = ranges(1, i+1);
        [sat_r_arr(i, 1:3), sat_v_arr(i, 1:3)] = Satellite_position_and_velocity(time, j);
    end

    r_aj_arr = zeros(n_sat, 1);
    r_aj_r_arr = zeros(n_sat, 1); 
    u = zeros(n_sat, 3);
    
    % Predict the ranges from the approximate user position to each satellite
    for m = 1:n_sat
        r_aj = ranges(epoch, m+1);
        r_ej = sat_r_arr(m , 1:3).';
        v_ej = sat_v_arr(m , 1:3).';
        for n = 1:2
            q = omega_ie * r_aj / c;
            C = [1 q 0; -q 1 0; 0 0 1];
     
            r_2 = C*r_ej - x_1(1:3);
            r_aj = sqrt(r_2.' * r_2);
        end
        q = omega_ie * r_aj / c;
        C = [1 q 0; -q 1 0; 0 0 1];
    
        u(m, 1:3) = (C*r_ej - x_1(1:3)) / r_aj;
        r_aj_arr(m) = r_aj;
        r_aj_r_arr(m) = u(m, 1:3) * (C * (v_ej + Omega_ie * r_ej) - (x_1(4:6) + Omega_ie * x_1(1:3)));
    end

    % Measurement matrix (Step 5)
    H = zeros(n_sat*2, 8);
    for m = 1:n_sat*2
        if m <= n_sat
            H(m, :) = [-u(m, 1) -u(m, 2) -u(m, 3) 0 0 0 1 0];
        else
            H(m, :) = [0 0 0 -u(m-n_sat, 1) -u(m-n_sat, 2) -u(m-n_sat, 3) 0 1];
        end
    end

    % Standard deviation
    % Pseudo-range measurements
    std_p = 10;
    % Pseudo-range-rate measurements
    std_r = 0.05;
    
    % Update Measurement noise covariance matrix (Step 6)
    R = eye(n_sat*2);
    for m = 1:n_sat*2
        if m <= n_sat
            R(m, m) = std_p^2;
        else
            R(m, m) = std_r^2;
        end
    end


    % Update the Kalman gain matrix (Step 7)
    K = P_1 * H.' / (H * P_1 * H.' + R);

    % Form the measurement innovation vector (Step 8)
    z = zeros(n_sat*2, 1);
    for m = 1:n_sat*2
        if m <= n_sat
            z(m) = ranges(epoch, m+1) - r_aj_arr(m) - x_1(7);
        else
            z(m) = range_rates(epoch, m-n_sat+1) - r_aj_r_arr(m-n_sat) - x_1(8);
        end
    end

    % ------------- OUTLIER DETECTION - residuals vector v -----------------
    I_m = eye(16, 16);
    v = (H * inv(H' * H) * H' - I_m) * z;

    % Compute the residuals covariance matrix Cv
    sigma_p = 10;  % measurement error standard deviation
    Cv = (I_m - H * inv(H' * H) * H') * sigma_p^2;

    %Compute the normalized residuals and detect outliers
    normalized_residuals = abs(v) ./ sqrt(diag(Cv));
    outliers = normalized_residuals > T;

    % If any outliers are detected, recalculate your position without the outliers
    if any(outliers)
        H(outliers, :) = [];
        R(outliers, :) = [];
        R(:, outliers) = [];  % Remove the columns corresponding to the outliers
        z(outliers) = [];  % Remove the outlier measurements

        % Recalculate the Kalman gain matrix without outliers
        K = P_1 * H.' / (H * P_1 * H.' + R);

        % Update the state estimate without outliers (Step 9)
        x_1 = x_1 + K * z;

        % Update the error covariance matrix without outliers (Step 10)
        P_1 = (eye(size(K,1)) - K * H) * P_1;
    else

    
        % Update the state estimate (Step 9)
        x_1 = x_1 + K * z;
    
        % Update the error covariance matrix (Step 10)
        P_1 = (eye(8) - K * H) * P_1;
    end

    % Convert the state estimate to NED coordinates 
    [lat,long,h,v] = pv_ECEF_to_NED(x_1(1:3),x_1(4:6));
    lat = round(lat * rad_to_deg,6);
    long = round(long * rad_to_deg,6);

    Pos_vel_NEC(end+1, :) = {(epoch-2)/2, lat, long, h, v(1), v(2), v(3)};

    % Update x_0 and P_0
    x_0 = x_1;
    P_0 = P_1;

end

Pos_vel_NEC(1, :) = []; % Remove the first row which is the header
writecell(Pos_vel_NEC, 'ans/CW_GNSS_Pos_Vel.csv');

end

function [x_k_est, P_k_est] = Initialize_Pos

    c = 299792458; % Speed of light in m/s
    omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
    Omega_ie = Skew_symmetric([0,0,omega_ie]);

    pseudo_ranges = csvread('data/Pseudo_ranges.csv');
    pseudo_range_rates = csvread('data/Pseudo_range_rates.csv');

    n_sat = size(pseudo_ranges, 2);

    %Compute positon and Velocity of Satellite 
    sat_r_arr = zeros(n_sat, 3);
    sat_v_arr = zeros(n_sat, 3);
    time = 0;
    for i = 1:n_sat-1
        j = pseudo_ranges(1, i+1);
        [sat_r_arr(i, 1:3), sat_v_arr(i, 1:3)] = Satellite_position_and_velocity(time, j);
    end

    r_eb_e = [0; 0; 0];
    c_offset = 1E-04;
    v_eb_e = [0; 0; 0];
    c_drift = 1E-04;
    
    % Predict the ranges from the approximate user position to each satellite
    while true

        r_aj_p = zeros(n_sat, 1);
        r_aj_r = zeros(n_sat, 1);
        u = zeros(n_sat, 3);
    
        for m = 1:n_sat-1
            r_aj = pseudo_ranges(2, m+1);
            r_ej = sat_r_arr(m, 1:3).';
            v_ej = sat_v_arr(m, 1:3).';
            for n = 1:2
                q = omega_ie * r_aj / c;
                C = [1 q 0; -q 1 0; 0 0 1];
                r_2 = C*r_ej - r_eb_e;
                r_aj = sqrt(r_2.' * r_2);
            end
            r_aj_p(m) = r_aj;
            % Compute unit vector
            u(m, 1:3) = (C*r_ej - r_eb_e) / r_aj;
            % Compute range rates
            r_aj_r(m) = u(m, 1:3) * (C * (v_ej + Omega_ie * r_ej) - (v_eb_e + Omega_ie * r_eb_e));
        end
    
        % Formulate predicted state vector, measurement innovation vector,
        % and measurement matrix
        x_r = [r_eb_e; c_offset];
        x_v = [v_eb_e; c_drift];
        z_r = zeros(8, 1);
        z_v = zeros(8, 1);
        H = ones(8, 4);
        for i = 1:8
            z_r(i) = pseudo_ranges(2, i+1) - r_aj_p(i) - c_offset;
            z_v(i) = pseudo_range_rates(2, i+1) - r_aj_r(i) - c_drift;
            H(i, :) = [-u(i, 1) -u(i, 2) -u(i, 3) 1];
        end
    
        
        x_r = x_r + (H.' * H) \ H.' * z_r;
        x_v = x_v + (H.' * H) \ H.' * z_v;
    
        r_eb_e_z = x_r(1:3);
        c_offset = x_r(4);
        v_eb_e_z = x_v(1:3);
        c_drift = x_v(4);
    
        limit = 0.1;
        if sqrt((r_eb_e(1) - r_eb_e_z(1))^2 +...
                (r_eb_e(2) - r_eb_e_z(2))^2 +...
                (r_eb_e(3) - r_eb_e_z(3))^2) < limit
            break
        end

        r_eb_e = r_eb_e_z;
        v_eb_e = v_eb_e_z;
    end

    std_p = 10;
    std_v = 0.05;
    std_co = 100000;
    std_cd = 200;

    x_k_est = [r_eb_e_z; v_eb_e_z; c_offset; c_drift];
    P_k_est = [std_p^2 0       0       0       0       0       0        0;...
               0       std_p^2 0       0       0       0       0        0;...
               0       0       std_p^2 0       0       0       0        0;...
               0       0       0       std_v^2 0       0       0        0;...
               0       0       0       0       std_v^2 0       0        0;...
               0       0       0       0       0       std_v^2 0        0;...
               0       0       0       0       0       0       std_co^2 0;...
               0       0       0       0       0       0       0        std_cd^2;];

end



