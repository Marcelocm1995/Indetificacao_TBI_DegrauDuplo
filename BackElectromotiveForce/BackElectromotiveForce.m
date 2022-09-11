%% Limpa as variaveis, exceto a tabela de log. Tambem fecha as janelas
clearvars;
clc;
close all;
datalog = csvread("C:\Users\marce\OneDrive\Documentos\USP\Indetificacao_TBI_DegrauDuplo\BackElectromotiveForce\AQCs\scope_03.csv");

%% separa as informacoes importadas em vetores 

%Scope Keysight  
offset = datalog(1,1);
time = datalog(:,1) + 0.12; 
BackEMF = datalog(:,2) * -1;
TPS_RAW = datalog(:,3);
TPS_max = 3.01; 
TPS_min = 0.343;
TPS_Deg = mapfun(TPS_RAW, TPS_min, TPS_max, 0, 90);

TPS_Rad = deg2rad(TPS_Deg);
% TPS_Rads = diff(TPS_Rad);
% TPS_Rads(2000) = TPS_Rads(1999);

for a = 1:49
index = find((time(:,1)>(a*0.009999)) & (time(:,1)<(a*0.010001)));
TPS_Rad100hz(a) = TPS_Rad(index);
Time_100hz(a) = time(index);
end

TPS_Rads = diff(TPS_Rad100hz);
TPS_Rads(49) = TPS_Rads(48);
TPS_Rads = TPS_Rads * 100; %to second

x = linspace(0, 0.5, 49);

%%

% time = time - 0.038;
% BackEMFFiltred = FilterWindowHammingFs10000Fc100_Order5(BackEMF);
TPS_Rads = TPS_Rads';
TPS_RadsFiltred = FilterWindowHammingFs10000Fc100_Order5(TPS_Rads);

figure;
plot(x, TPS_Rads, x-0.03, TPS_RadsFiltred);

figure;
plot(time, BackEMF, time, TPS_Rad, x+0.01-0.025, TPS_RadsFiltred);
grid on;
legend('Back Emf (Volts)', 'Position (Rad)', 'Velocity (Rad/s)');

%% Results
% Using t = 0.15s :
%   -> Back EMF = -9,5Volts
%   -> Angular Velocity = -7Rad/s
% Then Kb' is 0.56V.s/rad