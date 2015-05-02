clear all
close all
clc

cd('/home/amigo/ros/hydro/repos/https:/github.com/tue-robotics/stem_tracker/exp_truss_dangle')

% data = load('out_4.log');
% n = length(data);
% 
% dt = 1e-3;
% 
% t = linspace(0,dt*(n-1),n);
% plot(t,data(:,1))
% hold on
% plot(t,data(:,2),'r')
% 
% psd_data = data(950:6822,2);
% 
% h = spectrum.burg;
% Hpsd = psd(h,psd_data,'Fs',1/dt);
% figure
% plot(Hpsd)

Fs = 1000;   t = 0:1/Fs:.296;
x = cos(2*pi*t*1.2)+randn(size(t));
h = spectrum.welch;    % Create a Welch spectral estimator.

Hpsd = psd(h,x,'Fs',Fs);             % Calculate the PSD
plot(Hpsd)                           % Plot the PSD.