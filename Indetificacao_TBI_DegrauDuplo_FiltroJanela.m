%% Limpa as variaveis, exceto a tabela de log. Tambem fecha as janelas
% clearvars -except simout simu_gmf simu_PI;
clc;
% datalog = csvread("C:\Users\marce\OneDrive\Documentos\USP\Indetificacao_TBI_DegrauDuplo\Aquisitions\20072022_3volts_to_7volts.h");
% datalog = csvread("C:\Users\marce\OneDrive\Documentos\USP\Indetificacao_TBI_DegrauDuplo\Aquisitions\31072022_3volts_to_7volts.h");
datalog = csvread("C:\Users\marce\OneDrive\Documentos\USP\Indetificacao_TBI_DegrauDuplo\Aquisitions\31072022_3volts_to_7volts_amp.h");

close all;

%% separa as informacoes importadas em vetores 

index = find((datalog(:,3)>0));
datalog = datalog(index(1):end,:);

time = (datalog(:,1) - datalog(1,1)) / 1000;   % vetor de temp em S
RAD_S = (datalog(:,2) / 10);               
CONTROLE_PWM = (datalog(:,3) / 100) * 10;
Voltage_PWM = (datalog(:,4) / 100) * (12/3.2); 

RAW_ADC = (datalog(:,5) / 1);
vcc = 3.28;
ADC_Voltage = RAW_ADC * (vcc/4096);
Current = (4*ADC_Voltage / (vcc/3.3)) -   (4*(vcc/2) / (vcc/3.3));

%% Filters
RadF100_Order1 = FilterWindowHammingFs1000Fc100_Order1(RAD_S);
RadF10_Order1 = FilterWindowHammingFs1000Fc10_Order1(RAD_S);

CurrentF10_Order1 = FilterWindowHammingFs1000Fc10_Order1(Current);

figure;
% plot(time,RadF100_Order10);
hold on;
% plot(time,RadF50_Order10);
plot(time,RadF100_Order1);
% plot(time,RadF10_Order10);
plot(time,RadF10_Order1);
hold off;

%% Analise da Gma
s = tf('s');
k = 2.11; %2.35
tau = 0.038; %0.120
Gma= k/(tau*s+1);

figure;   
hold on;
plot(time,RAD_S);                   % plota os dados aquisitados
plot(time,CONTROLE_PWM);
plot(time,RadF10_Order1);
grid on;
grid minor;
hold off;
legend('Speed (Rad/s)', 'Step (Volts)', 'Filtred');

%%
figure;
plot(degrauduplo.Time, degrauduplo.Data(:,1),'b',time, RadF10_Order1,'r', time,CONTROLE_PWM, 'g');