% Remove variables, clear Command Window and close any figures
clear;
clc;
close all;

% Change these to properly locate the txt file you want to plot
filename = 'Omegas.txt';
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
Ymax = max(data(:));
Ymin = min(data(:));
axes.XLim = [0 iterations];
axes.YLim = [Ymin-0.5 Ymax+0.5];

scatter(Xaxis,Yaxis,'b','.')
scatter(XaxisRef,YaxisRef,'r','.')