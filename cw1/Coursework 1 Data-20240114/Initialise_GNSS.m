pos_est = [zeros(3,1);100000;];
vel_est = [zeros(3,1);200;];

time = 0;

pseudo_ranges = csvread('data/Pseudo_ranges.csv');
pseudo_range_rates = csvread('data/Pseudo_range_rates.csv');

Define_Constants;

sat_num = pseudo_ranges(1,2:9);
sat_idx = pseudo_ranges(1,:);
sat_time = pseudo_ranges(2:852,1);

P_matrix = zeros(8,8,length(sat_num));

sat_r_es_e = zeros(3, length(sat_num));
sat_v_es_e = zeros(3, length(sat_num));

% Compute positions and velocities for each satellite
for i = 1:length(sat_num)
    [position, velocity] = Satellite_position_and_velocity(time, sat_num(i));
    sat_r_es_e(:, i) = position;
    sat_v_es_e(:, i) = velocity;
end

sat_r_es_e = sat_r_es_e';
sat_v_es_e = sat_v_es_e';

    
for epo = 1:10
    
    r_aj_pred = zeros(length(sat_num),1);
    for j = 1:10
    for i = 1:length(sat_num)
        if j == 1
            r_aj_pred(i) = sqrt((eye(3)*sat_r_es_e(i,:)'-pos_est(1:3))'*(eye(3)*sat_r_es_e(i,:)'-pos_est(1:3)));
        end
        c_e_i =[1, omega_ie * r_aj_pred(i) / c, 0; -omega_ie * r_aj_pred(i)/c, 1, 0; 0, 0, 1];
        r_aj_pred(i) = sqrt((c_e_i*sat_r_es_e(i,:)'-pos_est(1:3))'*(c_e_i*sat_r_es_e(i,:)'-pos_est(1:3)));
    end
    end
    
    u_aj_e = zeros(length(sat_num),3); 
    for i=1:length(sat_num)
        c_e_i =[1, omega_ie * r_aj_pred(i) / c, 0; -omega_ie * r_aj_pred(i)/c, 1, 0; 0, 0, 1];
        u_aj_e(i,:) = (c_e_i*sat_r_es_e(i,:)'-pos_est(1:3))/r_aj_pred(i);
    end
    
    r_aj_diff = zeros(length(sat_num),1);
    for i = 1:length(sat_num)
        c_e_i =[1, omega_ie * r_aj_pred(i) / c, 0; -omega_ie * r_aj_pred(i)/c, 1, 0; 0, 0, 1];
        r_aj_diff(i) = u_aj_e(i,:)*(c_e_i*(sat_v_es_e(i,:)'+Omega_ie*sat_r_es_e(i,:)')-(vel_est(1:3)+Omega_ie*pos_est(1:3)));
    end
    
    delta_z_pos = pseudo_ranges(2,2:9)' - r_aj_pred - ones(8,1)*pos_est(4);
    delta_z_vel = pseudo_range_rates(2,2:9)' - r_aj_diff - ones(8,1)*vel_est(4);
    
    H_G_e = [-u_aj_e, ones(8,1)];
    
    pos_est = pos_est + (H_G_e'*H_G_e)\H_G_e'*delta_z_pos;
    vel_est = vel_est + (H_G_e'*H_G_e)\H_G_e'*delta_z_vel;
    
end

x_k_est = [pos_est(1:3);vel_est(1:3);pos_est(4);vel_est(4);];

P_k_est = zeros(8,8);
P_k_est(1,1) = 27.04;
P_k_est(2,2) = 27.04;
P_k_est(3,3) = 27.04;
P_k_est(4,4) = 0.0004;
P_k_est(5,5) = 0.0004;
P_k_est(6,6) = 0.0004;
P_k_est(7,7) = 10000000000;
P_k_est(8,8) = 4000;






