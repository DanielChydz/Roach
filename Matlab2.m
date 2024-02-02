clc
clear all
close all

% Odczyt danych z pliku
dataTable = abs(readmatrix('Data_20240202-190406.csv'));
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

% Create a time vector
timeVector = (0:(height(dataTable)-1)) * 0.05; % Assuming each row is 50ms apart

% Plot the data
figure('WindowState','maximized');
subplot(3, 1, 1);
grid on;
plot(timeVector, leftMotorPulses, 'LineWidth', 1);
hold on;
plot(timeVector, rightMotorPulses, 'LineWidth', 1);
yline(setPoint, ':', strcat('Wartosć zadana=', num2str(round(setPoint, 1))));
% Add labels and title
ylim([0 max([max(leftMotorPulses), max(rightMotorPulses), max(setPoint)])+max([max(leftMotorPulses), max(rightMotorPulses), max(setPoint)])*0.15]);
xlabel('Czas, s');
ylabel('Pozycja silników, cm');
title('Pozycja silników')
legend({'Lewy silnik', 'Prawy silnik'}, 'Location', 'southeast');

subplot(3, 1, 2);
grid on;
plot(timeVector, leftMotorSpeed);
hold on;
plot(timeVector, rightMotorSpeed);
% Add labels and title
ylim([0 max([max(leftMotorSpeed), max(rightMotorSpeed)])+10]);
xlabel('Czas, s');
ylabel('Prędkość silników, %');
yline(maxMotorSpeed, ':', 'Prędkość maksymalna');
title('Prędkość zadana silników');
legend({'Lewy silnik', 'Prawy silnik'}, 'Location', 'southeast');

subplot(3, 1, 3);
grid on;
plot(timeVector, leftMotorLoopPulses);
hold on;
plot(timeVector, rightMotorLoopPulses);
% Add labels and title
ylim([0 max([max(leftMotorLoopPulses), max(rightMotorLoopPulses), maxPulsesPerLoop])+0.2]);
xlabel('Czas, s');
ylabel('Prędkość silników, km/h');
yline(maxPulsesPerLoop, ':', strcat('Ograniczenie prędkości p_{max}=', num2str(round(maxPulsesPerLoop, 1)), ', km/h'));
yline(maxMotorSpeed*0.01*maxPulsesPerLoop, ':', strcat('Limit użytkownika=', num2str(round(maxMotorSpeed*0.01*maxPulsesPerLoop, 1)), ', km/h'));
title('Prędkość rzeczywista silników');
legend({'Lewy silnik', 'Prawy silnik'}, 'Location', 'southeast');