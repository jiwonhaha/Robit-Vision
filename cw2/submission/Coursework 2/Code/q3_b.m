% This script runs Q3(b)
clear
install_sparseinv
clc

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableGPS = false;
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Magic tuning for the no-prediction case
configuration.laserDetectionRange = 30;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q3_a');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Optimize every 500 timesteps to get a picture of the situation as it evolves
drivebotSLAMSystem.setRecommendOptimizationPeriod(500);

%prune Edges
drivebotSLAMSystem.setGraphPruning(true);

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
