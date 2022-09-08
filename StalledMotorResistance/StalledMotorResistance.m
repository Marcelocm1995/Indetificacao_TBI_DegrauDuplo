%% Limpa as variaveis, exceto a tabela de log. Tambem fecha as janelas
clearvars;
clc;
close all;
datalog = csvread("C:\Users\marce\OneDrive\Documentos\USP\Indetificacao_TBI_DegrauDuplo\StalledMotorResistance\AQCs\StalledMotorResistance01.dat");

%% separa as informacoes importadas em vetores 

% index = find((datalog(:,3)>0));
% datalog = datalog(index(1):end,:);

% time = (datalog(:,1) - datalog(1,1)) / 1000;   % vetor de temp em S     
time = datalog(:,1)/ 1000;   % vetor de temp em S  
CONTROLE_PWM = (datalog(:,2) / 100) * 10;
Voltage_PWM = (datalog(:,3) / 100) * (12/3.2); 
RAW_ADC = (datalog(:,4) / 1);
vcc = 3.38;
vref = 3.3;
ADC_Voltage = RAW_ADC * (vref/4096);
Current = mapfun(ADC_Voltage, 0.1*vcc, 0.9*vcc, -5, 5);

%%

CurrentF10_Order1 = FilterWindowHammingFs1000Fc10_Order1(Current);
figure;
plot(time, CONTROLE_PWM, time, Current, time, CurrentF10_Order1);

figure;
plot(CONTROLE_PWM, CurrentF10_Order1);
grid minor;
xlabel('Tensão (V)');
ylabel('Corrente (A)');
set(gca,'XTick',(0:1:12));
set(gca,'YTick',(-1:0.5:5));

Resistance(1,1) = CONTROLE_PWM(1000,1)/CurrentF10_Order1(1000,1);
Resistance(2,1) = CONTROLE_PWM(2000,1)/CurrentF10_Order1(2000,1);
Resistance(3,1) = CONTROLE_PWM(3000,1)/CurrentF10_Order1(3000,1);
Resistance(4,1) = CONTROLE_PWM(4000,1)/CurrentF10_Order1(4000,1);
Resistance(5,1) = CONTROLE_PWM(5000,1)/CurrentF10_Order1(5000,1);
Resistance(6,1) = CONTROLE_PWM(6000,1)/CurrentF10_Order1(6000,1);
Resistance(7,1) = CONTROLE_PWM(7000,1)/CurrentF10_Order1(7000,1);
Resistance(8,1) = CONTROLE_PWM(8000,1)/CurrentF10_Order1(8000,1);
Resistance(9,1) = CONTROLE_PWM(9000,1)/CurrentF10_Order1(9000,1);
Resistance(10,1) = CONTROLE_PWM(10000,1)/CurrentF10_Order1(10000,1);
Resistance(11,1) = CONTROLE_PWM(11000,1)/CurrentF10_Order1(11000,1);
Resistance(12,1) = CONTROLE_PWM(12000,1)/CurrentF10_Order1(12000,1);

ResistanceAvg = mean(Resistance)