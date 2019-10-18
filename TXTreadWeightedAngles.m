% Remove variables, clear Command Window and close any figures
clear;
clc;
close all;

% Change these to properly locate the txt files you want to plot
Angle_filename = 'Angles.txt';
Weight_filename = 'Weights.txt';
config_type = 'Release';
build_dir = 'build';
Angle_path = sprintf('%s/%s/%s', build_dir, config_type, Angle_filename);
Weight_path = sprintf('%s/%s/%s', build_dir, config_type, Weight_filename);

% Read txt
Angle_data = readmatrix(Angle_path);
Weight_data = readmatrix(Weight_path);

% Get matrix sizes
datasize = size(Angle_data);
iterations = datasize(1);
statenumber = datasize(2);

maxW = max(max(Weight_data));

hold on
for i = 1: iterations
    angles = Angle_data(i,:);
    weights = Weight_data(i,2:statenumber);
    
   for j = 1 : statenumber -1
       scatter(i, angles(1+j), '.', 'b', 'MarkerEdgeAlpha', weights(j)/maxW)
   end  
    
    scatter(i, angles(1), '.','r')
end
