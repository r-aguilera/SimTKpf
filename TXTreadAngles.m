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
Xaxis = []; XaxisRef = [];
Yaxis = []; YaxisRef = [];

% Assigning values
for i = 1 : iterations
    XaxisRef = [XaxisRef i];
    YaxisRef = [YaxisRef data(i,1)];
    Xaxis = [Xaxis,linspace(i,i,statenumber-1)];
    Yaxis = [Yaxis,data(i,2:statenumber)];
end

% Plot the values
hold on;
axes = gca;
axes.XLim = [0 iterations];
axes.YLim = [0 2*pi];
xlabel('Iteration')
ylabel('Angle [rad]')

scatter(Xaxis,Yaxis,'b','.')
scatter(XaxisRef,YaxisRef,'r','.')