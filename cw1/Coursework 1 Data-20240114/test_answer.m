clear,clc;

Initialise_GNSS;

epoch_output = zeros(length(sat_time),7);

tau_s = 0.5;

phi_k_1 = [     eye(3), tau_s*eye(3), zeros(3,1), zeros(3,1);
            zeros(3,3),       eye(3), zeros(3,1), zeros(3,1);
            zeros(1,3),   zeros(1,3),          1,      tau_s;
            zeros(1,3),   zeros(1,3),          0,          1;];
       
S_a_e  = 0.01;
S_c_a  = 0.01;
S_cf_a = 0.04;


Q_k_1 = [(1/3)*S_a_e*tau_s^3*eye(3), (1/2)*S_a_e*tau_s^2*eye(3),                       zeros(3,1),           zeros(3,1);
         (1/2)*S_a_e*tau_s^2*eye(3),         S_a_e*tau_s*eye(3),                       zeros(3,1),           zeros(3,1);
                         zeros(1,3),                 zeros(1,3), S_c_a*tau_s+(1/3)*S_cf_a*tau_s^3, (1/2)*S_cf_a*tau_s^2;
                         zeros(1,3),                 zeros(1,3),             (1/2)*S_cf_a*tau_s^2,         S_cf_a*tau_s;];


for iter = 1:length(sat_time)

% detect outlier and remove the wrong sat
% sat_num = outlier_detecter(x_k_est, P_k_est, phi_k_1, Q_k_1, sat_time, iter, pseudo_ranges, pseudo_range_rates, omega_ie, Omega_ie, c);
sat_num = pseudo_ranges(1,2:9);

x_k_pred = phi_k_1*x_k_est;

P_k_pred = phi_k_1*P_k_est*phi_k_1' + Q_k_1;

sat_r_es_e = zeros(3, length(sat_num));
sat_v_es_e = zeros(3, length(sat_num));

% Compute positions and velocities for each satellite
for i = 1:length(sat_num)
    [position, velocity] = Satellite_position_and_velocity(time, sat_num(i));
    sat_r_es_e(:, i) = position;
    sat_v_es_e(:, i) = velocity;
end

r_es_e = sat_r_es_e';
v_es_e = sat_v_es_e';


r_aj_pred = zeros(length(sat_num),1);

for j = 1:50
for i = 1:length(sat_num)
    if j == 1
        r_aj_pred(i) = sqrt((eye(3)*r_es_e(i,:)'-x_k_pred(1:3))'*(eye(3)*r_es_e(i,:)'-x_k_pred(1:3)));
    end
    c_e_i =[1, omega_ie * r_aj_pred(i) / c, 0; -omega_ie * r_aj_pred(i)/c, 1, 0; 0, 0, 1];
    r_aj_pred(i) = sqrt((c_e_i*r_es_e(i,:)'-x_k_pred(1:3))'*(c_e_i*r_es_e(i,:)'-x_k_pred(1:3)));   
end
end

u_aj_e = zeros(length(sat_num),3);
for i = 1:length(sat_num)
    c_e_i =[1, omega_ie * r_aj_pred(i) / c, 0; -omega_ie * r_aj_pred(i)/c, 1, 0; 0, 0, 1];
    u_aj_e(i,:) = (c_e_i*r_es_e(i,:)' - x_k_pred(1:3))/r_aj_pred(i);
end

r_aj_diff = zeros(length(sat_num),1); 
for i = 1:length(sat_num)
    c_e_i =[1, omega_ie * r_aj_pred(i) / c, 0; -omega_ie * r_aj_pred(i)/c, 1, 0; 0, 0, 1];
    r_aj_diff(i) = u_aj_e(i,:)*(c_e_i*(v_es_e(i,:)' + Omega_ie*r_es_e(i,:)') - (x_k_pred(4:6) + Omega_ie*x_k_pred(1:3)));
end

H_k = [                  -u_aj_e, zeros(length(sat_num),3),  ones(length(sat_num),1), zeros(length(sat_num),1);
        zeros(length(sat_num),3),                  -u_aj_e, zeros(length(sat_num),1),  ones(length(sat_num),1);];

sigma_p = 10;
sigma_r = 0.05;

R_k = blkdiag(sigma_p^2*eye(length(sat_num)), sigma_r^2*eye(length(sat_num)));

K_k = P_k_pred*H_k'*inv(H_k*P_k_pred*H_k' + R_k);

csv_idx = [];
for i=1:length(sat_num)
    idx = find(sat_idx==sat_num(i));
    csv_idx = [csv_idx, idx];
end

temp_1 = [pseudo_ranges(iter+1,csv_idx)'; pseudo_range_rates(iter+1,csv_idx)'];
temp_2 = [r_aj_pred;r_aj_diff;];
temp_3 = [x_k_pred(7)*ones(length(sat_num),1); x_k_pred(8)*ones(length(sat_num),1);];
delta_z = temp_1 - temp_2 - temp_3;

% detect outlier again, print outlier if it has outlier.
% temp = outlier_detection(delta_z, H_k, R_k, sat_num, sat_time(iter));

x_k_est = x_k_pred + K_k*delta_z;

P_k_est = (eye(8) - K_k*H_k)*P_k_pred;

[L_b,lambda_b,h_b,v_eb_n] = pv_ECEF_to_NED(x_k_est(1:3,1), x_k_est(4:6,1));

L_b = rad2deg(L_b);

lambda_b = rad2deg(lambda_b);

epoch_output(iter,:) = [sat_time(iter), L_b, lambda_b, h_b, v_eb_n'];

P_matrix(:,:,iter) = P_k_est;
    
end

csvwrite('aaaaaaa.csv',epoch_output)

plot(epoch_output(:,2),epoch_output(:,3))

writematrix(epoch_output, 'GNSS_Pos_Vel_NED.csv');
save('GNSS_P_matrix.mat', "P_matrix");

