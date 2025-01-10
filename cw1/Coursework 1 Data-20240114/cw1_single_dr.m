deg_to_rad = 0.01745329252; % Degrees to radians conversion factor
rad_to_deg = 1/deg_to_rad; % Radians to degrees conversion factor
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
Omega_ie = Skew_symmetric([0,0,omega_ie]);
R_0 = 6378137; 
e = 0.0818191908425; %WGS84 eccentricity

% Read GNSS result and dead reckoning data
dead_reckoning = csvread("data/Dead_reckoning.csv"); %#ok<CSVRD>
gnss = csvread("ans/CW_GNSS_Pos_Vel.csv"); %#ok<CSVRD>

% Number of epoches
epochs = size(dead_reckoning, 1);

lat = gnss(1, 2); % initial lattitude
long = gnss(1, 3); % initial longtitude
h = gnss(1, 4); % Geodetic height

% Calculate latitude, longitude, velocity and heading from initial position
% and dead reckoning
lawnmower_dr = Dead_Reckoning(lat, long, h, dead_reckoning);

filename = 'ans/Lawnmower_DR.csv';
writematrix(lawnmower_dr, filename);

% =========================================================================
% 
% Calculate latitude, longitude, velocity and heading from initial position
% and dead reckoning
function converted_dr = Dead_Reckoning(lat, long, h, dead_reckoning)
    
    deg_to_rad = 0.01745329252; % Degrees to radians conversion factor
    rad_to_deg = 1/deg_to_rad; % Radians to degrees conversion factor
    
    L = 0.5; % Wheel Base
    
    % Number of epoches
    epoches = size(dead_reckoning, 1);
    
    % Define an array of dead reckoning solution
    converted_dr = zeros(epoches, 6);

    % Convert latitude and longitude from degree to radian
    lat = lat * deg_to_rad;
    long = long * deg_to_rad;

    % Convert heading with Gyro-Magnetometor Integration
    heading = Gyro_Integration(dead_reckoning(:,6), dead_reckoning(:,7));

    
    for i = 1:epoches
        % Average wheel speeds for forward speed
        wheel_speeds = dead_reckoning(i, 2:5);
        v_forward = mean(wheel_speeds);
        % Compute lateral speed
        if v_forward == 0
            v_lat = 0;
        else
            delta = (dead_reckoning(i, 6) * L) / v_forward;
            v_lat = v_forward * tan(delta);
        end
        % Compute north velocity and east velocity
        v_n = v_forward * cos(heading(i)) - v_lat * sin(heading(i));
        v_e = v_forward * sin(heading(i)) + v_lat * cos(heading(i));
    
        % Compute north and east curvature
        [R_N,R_E] = Radii_of_curvature(lat);
        % Update latitude and longitude
        lat = lat + (v_n * 0.5 / (R_N + h));
        long = long + (v_e * 0.5 / ((R_E + h) * cos(lat)));

        % Save the data in the solution
        converted_dr(i, 1) = dead_reckoning(i, 1);
        converted_dr(i, 2) = lat * rad_to_deg;
        converted_dr(i, 3) = long * rad_to_deg;
        converted_dr(i, 4) = v_n;
        converted_dr(i, 5) = v_e;
        converted_dr(i, 6) = heading(i) * rad_to_deg;
    end

end


% =========================================================================
% 
% Convert heading with Gyro-Magnetometor Integration
function h_integrated = Gyro_Integration(gyro_rate, mag_heading)

    deg_to_rad = 0.01745329252; % Degrees to radians conversion factor
    rad_to_deg = 1/deg_to_rad; % Radians to degrees conversion factor
    
    % Number of epoches
    epoches = size(gyro_rate, 1);
    % Define an array of heading solution
    h_integrated = zeros(epoches, 1);
    % Define an array of heading from gyroscope
    gyro_heading = zeros(epoches, 1);
            
    t = 0.5; % time interval
    S_rg = 3E-06; % PSD of gyroscope measurement errors
    S_bgd = 0; % PSD of gyroscope bias errors
    std_b = 1 * deg_to_rad; % bias standard deviation
    std_g = 3E-06; % gyroscope heading error variance
    std_m = 3E-06; % magnetometor heading error variance
    
    % Compute heading from gyroscope
    gyro_heading(1) = mag_heading(1) * deg_to_rad;
    for epoch = 2:epoches
        gyro_heading(epoch) = gyro_heading(epoch - 1) + gyro_rate(epoch) * t;
    end
        
    % transition matrix
    T = [1 t;
         0 1];
        
    %  system noise covariance matrix
    Q = [(S_rg*t)+(S_bgd*t^3)/3 (S_bgd*t^2)/2;
         (S_bgd*t^2)/2          S_bgd*t];
        
    % Measurement Matrix
    H = [-1  0;
          0  -1];
        
     % Measurement Noise Covariance Matrix
    R = [std_m^2 0;
         0       std_m^2];
        
    % Initialize state filter
    x = [0; 0];
    % Initialize state estimation error covariance matrix 
    P = [std_g^2 0;
         0       std_b^2];
    
    % Kalman filter measurement
    for epoch = 1:epoches
        
        % Propagate state
        x = T * x;
        P = T * P * T.' + Q;
        
        % Kalman gain matrix
        K = P * H.' \ (H * P * H.' + R);
        
        % Measurement innovation
        z = [(mag_heading(epoch)*deg_to_rad - gyro_heading(epoch)); 0] - H * x;
        
        % Update state
        x = x + K * z;
        P = (eye(2) - K * H) * P;
        
        % Save the data in the solution
        h_integrated(epoch) = (gyro_heading(epoch) - x(1));
    end

end
