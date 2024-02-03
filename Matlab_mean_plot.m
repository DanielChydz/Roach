clc
clear all
close all

%% odczyt danych

files = dir(fullfile('1,5m30p', '*.csv'));
maxRows = 0;
% znalezienie pliku o największej liczbie rzędów (do inicjalizacji)
for i=1:5
    path = fullfile(files(i).folder, files(i).name);
    temp = readmatrix(path);
    rows = size(temp, 1);
    if rows > maxRows
        maxRows = rows;
    end
end

measurementsData = zeros(maxRows, 9, 5);
% odczytanie danych pomiarowych
for i=1:5
    temp = readmatrix(fullfile(files(i).folder, files(i).name));
    rows = size(temp, 1);
    measurementsData(1:rows, :, i) = temp;
end

%% wykresy

avgMatrix = zeros(maxRows, 9);
% obliczanie średniej
for col=1:9
    for row=1:maxRows
        avgMatrix(row, col) = mean(measurementsData(row, col, :), 3);
    end
end

setPoint = measurementsData(1, 1);
leftMotorLoopPulses = measurementsData(:, 2);
rightMotorLoopPulses = measurementsData(:, 3);
leftMotorPulses = measurementsData(:, 4);
rightMotorPulses = measurementsData(:, 5);
leftMotorSpeed = measurementsData(:, 6);
rightMotorSpeed = measurementsData(:, 7);
maxMotorSpeed = measurementsData(1, 8);
maxPulsesPerLoop = measurementsData(1, 9);

% zmiana jednostki z pulsów na cm
pulsesPerCm = 2800 / (2 * pi * 1.5);
setPoint = setPoint / pulsesPerCm;
leftMotorLoopPulses = leftMotorLoopPulses / pulsesPerCm * 20 * 60 * 60 / 100 / 1000;
rightMotorLoopPulses = rightMotorLoopPulses / pulsesPerCm * 20 * 60 * 60 / 100 / 1000;
leftMotorPulses = leftMotorPulses / pulsesPerCm;
rightMotorPulses = rightMotorPulses / pulsesPerCm;
maxPulsesPerLoop = maxPulsesPerLoop / pulsesPerCm * 20 * 60 * 60 / 100 / 1000;

% tworzenie wektora czasu
lastMeasurement = find(measurementsData(:,1) > 1);
lastMeasurement = lastMeasurement(end);
timeVector = (0:(lastMeasurement-1)) * 0.05; % 50 ms odstępu między iteracjami

% Plot the data
figure('WindowState','maximized');
grid on;
plot(timeVector, leftMotorPulses(1:lastMeasurement, 1), 'LineWidth', 1);
hold on;
plot(timeVector, rightMotorPulses(1:lastMeasurement, 1), 'LineWidth', 1);
yline(setPoint, ':', strcat('Wartosć zadana=', num2str(round(setPoint, 1))));
ylim([0 max([max(leftMotorPulses), max(rightMotorPulses), max(setPoint)])+max([max(leftMotorPulses), max(rightMotorPulses), max(setPoint)])*0.15]);
xlabel('Czas, s');
ylabel('Pozycja kół, cm');
legend({'Lewe koło', 'Prawe koło'}, 'Location', 'best');

figure('WindowState','maximized');
grid on;
plot(timeVector, leftMotorSpeed(1:lastMeasurement, 1), 'LineWidth', 1);
hold on;
plot(timeVector, rightMotorSpeed(1:lastMeasurement, 1), 'LineWidth', 1);
ylim([0 max([max(leftMotorSpeed), max(rightMotorSpeed)])+10]);
xlabel('Czas, s');
ylabel('Prędkość zadana kół, %');
yline(maxMotorSpeed, ':', strcat('Prędkość maksymalna=', maxMotorSpeed));
legend({'Lewe koło', 'Prawe koło'}, 'Location', 'best');

figure('WindowState','maximized');
grid on;
plot(timeVector, leftMotorLoopPulses(1:lastMeasurement, 1), 'LineWidth', 1);
hold on;
plot(timeVector, rightMotorLoopPulses(1:lastMeasurement, 1), 'LineWidth', 1);
ylim([0 max([max(leftMotorLoopPulses), max(rightMotorLoopPulses), maxPulsesPerLoop])+0.2]);
xlabel('Czas, s');
ylabel('Prędkość rzeczywista kół, km/h');
yline(maxPulsesPerLoop, ':', strcat('Ograniczenie prędkości p_{max}=', num2str(round(maxPulsesPerLoop, 1)), ', km/h'));
yline(maxMotorSpeed*0.01*maxPulsesPerLoop, ':', strcat('Limit użytkownika=', num2str(round(maxMotorSpeed*0.01*maxPulsesPerLoop, 1)), ', km/h'));
legend({'Lewe koło', 'Prawe koło'}, 'Location', 'best');

%set(gca,'fontsize', 32)