clc
clear all
close all

% Read the CSV file using readtable
dataTable = readtable('roachPythonScript/Roach/Data.csv');

% Extract columns
leftMotorPulses = abs(dataTable.Var1);
rightMotorPulses = abs(dataTable.Var2);
setPoint = abs(dataTable.Var3);
leftMotorSpeed = abs(dataTable.Var4);
rightMotorSpeed = abs(dataTable.Var5);

% Create a time vector
timeVector = (0:(height(dataTable)-1)) * 0.05; % Assuming each row is 50ms apart

% Plot the data
figure('WindowState','maximized');
subplot(2, 1, 1)
grid on
plot(timeVector, leftMotorPulses, 'LineWidth', 1);
hold on;
plot(timeVector, rightMotorPulses, 'LineWidth', 1);
yline(setPoint, ':', 'Wartość zadana')

% Add labels and title
ylim([0 max([max(leftMotorPulses), max(rightMotorPulses), max(setPoint)]) + 2000]);
xlabel('Czas, s')
ylabel('Liczba impulsów enkodera, -')
title('Położenie silników')
legend({'Położenie lewego silnika', 'Położenie prawego silnika'}, 'Location', 'southeast')


subplot(2, 1, 2)
grid on
plot(timeVector, leftMotorSpeed);
hold on
plot(timeVector, rightMotorSpeed);

% Add labels and title
ylim([0 max([max(leftMotorSpeed), max(rightMotorSpeed)])+10]);
xlabel('Czas, s')
ylabel('Prędkość silników, %')
title('Prędkość silników')
legend({'Prędkość lewego silnika', 'Prędkość prawego silnika'}, 'Location', 'northeast')
