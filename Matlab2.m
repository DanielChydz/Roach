clc
clear all
close all

% Odczyt danych z pliku
dataTable = abs(readmatrix('Data.csv'));
setPoint = dataTable(1, 1);
leftMotorLoopPulses = dataTable(:, 2);
rightMotorLoopPulses = dataTable(:, 3);
leftMotorPulses = dataTable(:, 4);
rightMotorPulses = dataTable(:, 5);
leftMotorSpeed = dataTable(:, 6);
rightMotorSpeed = dataTable(:, 7);
maxMotorSpeed = dataTable(1, 8);
maxPulsesPerLoop = dataTable(1, 9);

% Create a time vector
timeVector = (0:(height(dataTable)-1)) * 0.05; % Assuming each row is 50ms apart

% Plot the data
figure('WindowState','maximized');
subplot(3, 1, 1);
grid on;
plot(timeVector, leftMotorPulses, 'LineWidth', 1);
hold on;
plot(timeVector, rightMotorPulses, 'LineWidth', 1);
yline(setPoint, ':', 'Wartość zadana');
% Add labels and title
ylim([0 max([max(leftMotorPulses), max(rightMotorPulses), max(setPoint)])+max([max(leftMotorPulses), max(rightMotorPulses), max(setPoint)])*0.1]);
xlabel('Czas, s');
ylabel('Pozycja silników, pulsy');
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
ylim([0 max([max(leftMotorLoopPulses), max(rightMotorLoopPulses), maxPulsesPerLoop])+100]);
xlabel('Czas, s');
ylabel('Prędkość silników, pulsy/pętla');
yline(maxPulsesPerLoop, ':', 'Ograniczenie ilości pulsów');
title('Prędkość rzeczywista silników, pętla=50 ms, 1 obrót koła=2800 pulsów');
legend({'Lewy silnik', 'Prawy silnik'}, 'Location', 'southeast');