clc
clear all
close all

% odczyt danych z pliku
dataTable = abs(readmatrix('PID/P/Kp0.006.csv'));
setPoint = dataTable(1, 1);
leftMotorLoopPulses = dataTable(:, 2);
rightMotorLoopPulses = dataTable(:, 3);
leftMotorPulses = dataTable(:, 4);
rightMotorPulses = dataTable(:, 5);
leftMotorSpeed = dataTable(:, 6);
rightMotorSpeed = dataTable(:, 7);
maxMotorSpeed = dataTable(1, 8);
maxPulsesPerLoop = dataTable(1, 9);

% zmiana jednostki z pulsów na cm
pulsesPerCm = 2800 / (2 * pi * 1.5);
setPoint = setPoint / pulsesPerCm;
leftMotorLoopPulses = leftMotorLoopPulses / pulsesPerCm * 20 * 60 * 60 / 100 / 1000;
rightMotorLoopPulses = rightMotorLoopPulses / pulsesPerCm * 20 * 60 * 60 / 100 / 1000;
leftMotorPulses = leftMotorPulses / pulsesPerCm;
rightMotorPulses = rightMotorPulses / pulsesPerCm;
maxPulsesPerLoop = maxPulsesPerLoop / pulsesPerCm * 20 * 60 * 60 / 100 / 1000;

% tworzenie wektora czasu
timeVector = (0:(height(dataTable)-1)) * 0.05; % 50 ms odstępu między iteracjami

% Plot the data
figure('WindowState','maximized');
subplot(3, 1, 1);
grid on;
plot(timeVector, leftMotorPulses, 'LineWidth', 1);
hold on;
plot(timeVector, rightMotorPulses, 'LineWidth', 1);
yline(setPoint, ':', strcat('Wartosć zadana=', num2str(round(setPoint, 1))));
ylim([0 max([max(leftMotorPulses), max(rightMotorPulses), max(setPoint)])*1.15]);
xlabel('Czas, s');
ylabel('Pozycja silników, cm');
title('Pozycja silników')
legend({'Lewy silnik', 'Prawy silnik'}, 'Location', 'best');
set(gca,'fontsize', 18)

subplot(3, 1, 2);
grid on;
plot(timeVector, leftMotorSpeed, 'LineWidth', 1);
hold on;
plot(timeVector, rightMotorSpeed, 'LineWidth', 1);
ylim([0 max([max(leftMotorSpeed), max(rightMotorSpeed)])+10]);
xlabel('Czas, s');
ylabel('Prędkość silników, %');
yline(maxMotorSpeed, ':', 'Prędkość maksymalna');
title('Prędkość zadana silników');
legend({'Lewy silnik', 'Prawy silnik'}, 'Location', 'best');
set(gca,'fontsize', 18)

subplot(3, 1, 3);
grid on;
plot(timeVector, leftMotorLoopPulses, 'LineWidth', 1);
hold on;
plot(timeVector, rightMotorLoopPulses, 'LineWidth', 1);
ylim([0 max([max(leftMotorLoopPulses), max(rightMotorLoopPulses), maxPulsesPerLoop])+0.2]);
xlabel('Czas, s');
ylabel('Prędkość silników, km/h');
yline(maxPulsesPerLoop, ':', strcat('Ograniczenie prędkości p_{max}=', num2str(round(maxPulsesPerLoop, 1)), ', km/h'));
yline(maxMotorSpeed*0.01*maxPulsesPerLoop, ':', strcat('Limit użytkownika=', num2str(round(maxMotorSpeed*0.01*maxPulsesPerLoop, 1)), ', km/h'));
title('Prędkość rzeczywista silników');
legend({'Lewy silnik', 'Prawy silnik'}, 'Location', 'best');
set(gca,'fontsize', 18)