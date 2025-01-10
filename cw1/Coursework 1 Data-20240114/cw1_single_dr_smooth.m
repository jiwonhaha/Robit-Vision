
% ===============================================================
% 
% Coursework 1
% 
% ===============================================================

% Constants
deg_to_rad = 0.01745329252; % Degrees to radians conversion factor
rad_to_deg = 1/deg_to_rad; % Radians to degrees conversion factor
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
Omega_ie = Skew_symmetric([0,0,omega_ie]);
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity

% Read GNSS result and dead reckoning data
dead_reckoning = csvread("data/Dead_reckoning.csv"); %#ok<CSVRD>
% gnss = csvread("data/CW_GNSS_KF_Results.csv"); %#ok<CSVRD>

% Number of epoches
epochs = size(dead_reckoning, 1);

corrected_dr = zeros(epochs, 7);

h = 37.4; % Geodetic height
lat = 51.509254461345272; % initial lattitude
long = -0.161045484902720; % initial longtitude

lawnmower_dr = Dead_Reckoning(lat, long, h, dead_reckoning);

filename = 'data/Lawnmower_DR.csv';
writematrix(lawnmower_dr, filename);

% End of multiple epochs processing


function converted_dr = Dead_Reckoning(lat, long, h, dead_reckoning)
    
    deg_to_rad = 0.01745329252; % Degrees to radians conversion factor
    rad_to_deg = 1/deg_to_rad; % Radians to degrees conversion factor
    
    L = 0.5; % Wheel Base
    
    epoches = size(dead_reckoning, 1);
    
    converted_dr = zeros(epoches, 6);
    
    lat = lat * deg_to_rad;
    long = long * deg_to_rad;

    % Define the standard deviations for the errors
    wheel_speed_scale_error_std = 0.03; % 3% scale factor error
    wheel_speed_noise_std = 0.05; % Noise standard deviation
    gyro_bias_std = 1 * deg_to_rad / 3600; % Bias standard deviation (converted to rad/s)
    gyro_scale_error_std = 0.01; % 1% scale factor error
    gyro_cross_coupling_std = 0.001; % 0.1% cross-coupling error
    gyro_quantization_level = 2e-4; % Quantization level in radians per second

    
    for i = 1:epoches
        % Simulate wheel speed sensor errors
        wheel_speeds = dead_reckoning(i, 2:5);
        scale_factor = 1 + randn(1, 4) * wheel_speed_scale_error_std;
        noise = randn(1, 4) * wheel_speed_noise_std;
        quantized_wheel_speeds = round((wheel_speeds .* scale_factor + noise) / 0.02) * 0.02;
        
        
        v_forward = mean(quantized_wheel_speeds);
        if v_forward == 0
            v_lat = 0;
        else
            delta = (dead_reckoning(i, 6) * L) / v_forward;
            v_lat = v_forward * tan(delta);
        end
    
        mag_heading = dead_reckoning(i, 7) * deg_to_rad;
        gyro_err_std = 10^-4;
        mag_heading_err_std = 4 * deg_to_rad;
        heading_weight = (gyro_err_std*0.5)/ mag_heading_err_std;

        if i ==1
            heading_rad = mag_heading;

        else
            heading_rad = mag_heading * heading_weight + (1-heading_weight) * (converted_dr(i-1,6) * deg_to_rad+ dead_reckoning(i,6) * 0.5);
        end

        heading_deg = heading_rad * rad_to_deg;

%         v_n=(cos(heading_rad)-0.5*mag_heading* ...
%         0.5*sin(heading_rad))*v_forward*0.5 + (cos(heading_rad)* ...
%         0.2+sin(heading_rad)*0.2)*0.5*mag_heading;
%         
%         v_e=(sin(heading_rad)+0.5*mag_heading* ...
%         0.5*cos(heading_rad))*v_forward*0.5 + (sin(heading_rad)* ...
%         0.2-cos(heading_rad)*0.2)*0.5*mag_heading;
    
         v_n = v_forward * cos(heading_rad) - v_lat * sin(heading_rad);
         v_e = v_forward * sin(heading_rad) + v_lat * cos(heading_rad);

    
        [R_N,R_E] = Radii_of_curvature(lat);
    
        lat = lat + (v_n * 0.5 / (R_N + h));
        long = long + (v_e * 0.5 / ((R_E + h) * cos(lat)));
    
        converted_dr(i, 1) = dead_reckoning(i, 1);
        converted_dr(i, 2) = lat * rad_to_deg;
        converted_dr(i, 3) = long * rad_to_deg;
        converted_dr(i, 4) = v_n;
        converted_dr(i, 5) = v_e;
        converted_dr(i, 6) = heading_deg;
    end

end
