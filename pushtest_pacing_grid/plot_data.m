clc;close all;

%%
clear all;
load('results_pacing000.mat');
angle = results(1:13, 1);
mag = results(1:13, 2);
mag_x = cos(angle) .* mag;
mag_y = sin(angle) .* mag;                  

subplot(2,2,1)
plot(mag_x, mag_y, "k", "LineWidth", 1.5)
title("0.000s")
%%
clear all;
load('results_pacing0075.mat')
angle = results(1:13, 1);
mag = results(1:13, 2);
mag_x = cos(angle) .* mag;
mag_y = sin(angle) .* mag;                  

subplot(2,2,2)
plot(mag_x, mag_y, "k", "LineWidth", 1.5)
title("0.075s")
%% 
clear all;
load('results_pacing015.mat')
angle = results(1:13, 1);
mag = results(1:13, 2);
mag_x = cos(angle) .* mag;
mag_y = sin(angle) .* mag;                  

subplot(2,2,3)
plot(mag_x, mag_y, "k", "LineWidth", 1.5)
title("0.150s")
%% 
clear all;
load('result_pacing0225.mat')
angle = results(1:13, 1);
mag = results(1:13, 2);
mag_x = cos(angle) .* mag;
mag_y = sin(angle) .* mag;                  

subplot(2,2,4)
plot(mag_x, mag_y, "k", "LineWidth", 1.5)
title("0.225s")