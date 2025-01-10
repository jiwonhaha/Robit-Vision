
% Plot graphs
% =================================================================

lawnmower_dr = csvread("ans/Corrected_DR.csv"); %#ok<CSVRD>
kalman_gnss = csvread("ans/CW_GNSS_KF_Results.csv"); %#ok<CSVRD>

% Lattitude - Longtitude
x1 = lawnmower_dr(:, 2);
y1 = lawnmower_dr(:, 3);
x2 = kalman_gnss(:, 2);
y2 = kalman_gnss(:, 3);

x1_start = lawnmower_dr(1, 2);
y1_start = lawnmower_dr(1, 3);
x2_start = kalman_gnss(1, 2);
y2_start = kalman_gnss(1, 3);

x1_end = lawnmower_dr(end, 2);
y1_end = lawnmower_dr(end, 3);
x2_end = kalman_gnss(end, 2);
y2_end = kalman_gnss(end, 3);

plot(x1, y1, 'g', x2, y2, 'r')
title('Lattitude - Longtitude')
xlabel('Lattitude (degree)')
ylabel('Longtitude (degree)')
hold on  
plot(x1_start,y1_start,'g*')
plot(x1_end,y1_end,'g*')
plot(x2_start,y2_start,'ro')
plot(x2_end,y2_end,'ro')
hold off

saveas(gcf,'ans/Lat_Long.png')

% Heading
x = lawnmower_dr(:, 1);
y1 = lawnmower_dr(:, 6);
y2 = kalman_gnss(:, 6);

subplot(1,2,1)
plot(x, y1, 'g')
title('Dead Reckoning')
xlabel('Time (s)')
ylabel('Heading (degree)')

subplot(1,2,2)
plot(x, y2, 'r')
title('GNSS Kalman Filter')
xlabel('Time (s)')
ylabel('Heading (degree)')

saveas(gcf,'ans/Heading.png')

% North Velocity
x = lawnmower_dr(:, 1);
y1 = lawnmower_dr(:, 4);
y2 = kalman_gnss(:, 4);

subplot(1,2,1)
plot(x, y1, 'g')
title('Dead Reckoning')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

subplot(1,2,2)
plot(x, y2, 'r')
title('GNSS Kalman Filter')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

saveas(gcf,'ans/NorthV.png')

% East Velocity
x = lawnmower_dr(:, 1);
y1 = lawnmower_dr(:, 5);
y2 = kalman_gnss(:, 5);

subplot(1,2,1)
plot(x, y1, 'g')
title('Dead Reckoning')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

subplot(1,2,2)
plot(x, y2, 'r')
title('GNSS Kalman Filter')
xlabel('Time (s)')
ylabel('Velocity (m/s)')

saveas(gcf,'ans/NorthE.png')

