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

TPS_Rads = diff(TPS_Rad);

%%

% time = time - 0.038;
% CurrentFiltred = FilterWindowHammingFs10000Fc100_Order5(Current);

figure;
plot(time, BackEMF, time, TPS_Rad, time, TPS_Rads);
legend('Back Emf (Volts)', 'Position (Rad)');

% figure
% plot(time, Current, time-0.0003, CurrentFiltred, time, CONTROLE_PWM);
% 
% figure
% plot(time, CurrentFiltred, time, CONTROLE_PWM);
% 
% grid minor;
% xlabel('Tempo (S)');
% ylabel('Corrente (A)');
% set(gca,'XTick',(0:0.001:1));
% set(gca,'YTick',(-1:0.5:5));
