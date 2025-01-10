% This script runs Q2(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = false;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
xlabel('Time Step')
ylabel('Optimization times')
hold on

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')
xlabel('Time Step')
ylabel('Errors')
legend({'x error', 'y error', 'theta error'});

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
xlabel('Time Step')
ylabel('Vehicle Covariances')
legend({'x covariance', 'y covariance', 'theta covariance'});
hold on

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
xlabel('Time Step')
ylabel('Errors')
legend({'x error', 'y error', 'theta error'});
hold on

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
xlabel('Time Step')
ylabel('chi2 values')
hold on


% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();

% Work out the number of vehicle poses and landmarks. 
numVehicleVertices = 0;
numLandmarks = 0;

landmarkObservationsPerVehicleVertex = 0;
observationsPerLandmarkVertex = 0;

% Q2c:
% Finish implementing the code to capture information about the graph
% structure.

% Number of landmark edges
numEdgesLandmarks = 0;
for i = 1:length(allVertices)

    % If the current vertex is a landmark
    if (isa(allVertices{i}, 'drivebot.graph.LandmarkStateVertex'))
        % Get the landmark edges
        edgesList = edges(allVertices{i});

        % The number of landmarks initalized
        numLandmarks = numLandmarks + 1;
        numEdgesLandmarks = numEdgesLandmarks + length(edgesList);
    else
        % The number of vehicle poses stored
        numVehicleVertices = numVehicleVertices + isa(allVertices{i}, 'drivebot.graph.VehicleStateVertex');
    end
end
% The average number of observations made by a robot at each timestep
landmarkObservationsPerVehicleVertex = numEdgesLandmarks / numVehicleVertices;
% The average number of observations received by each landmark
observationsPerLandmarkVertex = numEdgesLandmarks / numLandmarks;


fprintf('The number of vehicle poses stored: %d\n', numVehicleVertices);
fprintf('The number of landmarks initalized: %d\n', numLandmarks);
fprintf('The average number of observations made by a robot at each timestep: %d\n', landmarkObservationsPerVehicleVertex);
fprintf('The average number of observations received by each landmark: %d\n', observationsPerLandmarkVertex);
