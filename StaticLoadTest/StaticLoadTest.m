%% Limpa as variaveis, exceto a tabela de log. Tambem fecha as janelas
clearvars;
clc;
close all;
datalog = csvread("C:\Users\marce\OneDrive\Documentos\USP\Indetificacao_TBI_DegrauDuplo\StalledMotorIndutance\AQCs\scope_13.csv");

%% separa as informacoes importadas em vetores 

% index = find((datalog(:,3)>0));
% datalog = datalog(index(1):end,:);

%nucleo f446re
% % time = (datalog(:,1) - datalog(1,1)) / 1000;   % vetor de temp em S     
% time = (datalog(:,1) - 2445 - 821)/ 1000;   % vetor de temp em S  
% CONTROLE_PWM = (datalog(:,2) / 100) * 10;
% RAW_ADC = (datalog(:,3) / 1);
% vcc = 3.36;
% vref = 3.3;
% ADC_Voltage = RAW_ADC * (vref/4096);
% Current = mapfun(ADC_Voltage, 0.1*vcc, 0.9*vcc, -5, 5);

%Scope Keysight  
vcc = 3.45;
offset = datalog(1,1);
time = datalog(:,1) + 0.1; 
ADC_Voltage = datalog(:,2);
CONTROLE_PWM = datalog(:,3);
Current = mapfun(ADC_Voltage, 0.1*vcc, 0.9*vcc, -5, 5);

%%

time = time - 0.038;
CurrentFiltred = FilterWindowHammingFs10000Fc100_Order5(Current);

figure;
plot(time, Current, time, CONTROLE_PWM);

figure
plot(time, Current, time-0.0003, CurrentFiltred, time, CONTROLE_PWM);

figure
plot(time, CurrentFiltred, time, CONTROLE_PWM);

grid minor;
xlabel('Tempo (S)');
ylabel('Corrente (A)');
set(gca,'XTick',(0:0.001:1));
set(gca,'YTick',(-1:0.5:5));

% WITHOUT DELAY
% 63% of steady state was 0,3mS, then Tau = t
% Therefore, the motor inductance was obtained as La = tau*Rt
% -> La = 1,1(mS) * 6.44(Ohm) = 7,08 (mH)
% -> La = 0,3(mS) * 6.44(Ohm) = 1,9 (mH)

% WITH DELAY
% 63% of steady state was 1,7mS, then Tau = t
% Therefore, the motor inductance was obtained as La = tau*Rt
% -> La = 1,7(mS) * 6.44(Ohm) = 10,95 (mH)