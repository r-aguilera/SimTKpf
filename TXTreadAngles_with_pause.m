% Remove variables, clear Command Window and close any figures
clear;
clc;
close all;

% Change these to properly locate the txt file you want to plot
filename = 'Angles.txt';
config_type = 'Release';
build_dir = 'build';
path = sprintf('%s/%s/%s', build_dir, config_type, filename);

% Read txt
data = readmatrix(path);

% Get matrix sizes
datasize = size(data);
iterations = datasize(1);
statenumber = datasize(2);

% Define axis variables
Xaxis = 1:iterations;
YaxisRef = data(:,1);

% Save figure axes handle
axes = gca;

% Plot each particle
for i = 2 : statenumber
    YaxisPt = data(:,i);
    scatter(Xaxis,YaxisRef,1,'r','.')
    hold on;
    axes.XLim = [0 iterations];
    axes.YLim = [0 2*pi];
    scatter(Xaxis,YaxisPt,'b','.')
    hold off;
    pause(0.01);
end

