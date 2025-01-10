% Read the CSV file without headers
filePath = 'ans/Corrected_DR.csv'; % Update with your file path as necessary
data = readmatrix(filePath);

% Extract the data
time = data(:, 1); % Time in seconds
latitude = data(:, 2); % Geodetic latitude in degrees
longitude = data(:, 3); % Geodetic longitude in degrees
northVelocity = data(:, 4); % North velocity in meters per second
eastVelocity = data(:, 5); % East velocity in meters per second
heading = data(:, 6); % Heading in degrees

% Plot the positions on a map
if exist('geoplot', 'file')
    figure
    geoplot(latitude, longitude, 'r-', 'LineWidth', 2)
    hold on
    
    % Optional: Add starting point
    geoscatter(latitude(1), longitude(1), 100, 'g', 'filled', 'MarkerEdgeColor', 'k');
    
    % Optional: Add ending point
    geoscatter(latitude(end), longitude(end), 100, 'b', 'filled', 'MarkerEdgeColor', 'k');
    
    title('Motion Path')
    geobasemap('satellite') % You can change the basemap as needed
else
    fprintf('Mapping Toolbox not available. Consider installing it for map plotting features.\n');
end

% Plot velocity vectors (optional)
% For simplicity, this plots vectors every n points to avoid clutter
n = 10; % Adjust n for fewer vectors
quiverLat = latitude(1:n:end);
quiverLon = longitude(1:n:end);
quiverU = eastVelocity(1:n:end);
quiverV = northVelocity(1:n:end);

% Convert geographic to Cartesian coordinates for quiver plot
[x, y] = geodetic2enu(quiverLat, quiverLon, zeros(size(quiverLat)), mean(latitude), mean(longitude), 0, wgs84Ellipsoid());

if exist('quiver', 'file')
    figure
    quiver(x, y, quiverU, quiverV)
    xlabel('East displacement (m)')
    ylabel('North displacement (m)')
    title('Velocity Vectors')
else
    fprintf('Consider plotting velocity vectors on a map for better visualization.\n');
end
