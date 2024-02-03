clc
clear all
close all

% inicjalizacja zmiennych
fileFolder = 'stat';
filePrefix = 'Data_';
fileExtension = '.csv';
startRow = 5;
endRow = 24;
rows = 10:24;
speedIndexes = 0:5:100;

% inicjalizacja wektorów do przechowywania średnich wartości
lewySilnik = zeros(size(speedIndexes));
prawySilnik = zeros(size(speedIndexes));

% Obliczenie średnich wartości dla każdego pliku
for idx = 1:length(speedIndexes)
    x = speedIndexes(idx);
    fileName = [fileFolder, '/', filePrefix, num2str(x), fileExtension];
    data = abs(readmatrix(fileName));
    lewySilnik(idx) = mean(data(rows, 1));
    prawySilnik(idx) = mean(data(rows, 2));
end

pulsesPerCm = 2800 / (2 * pi * 1.5);
lewySilnik = lewySilnik / pulsesPerCm * 20 * 60 * 60 / 100 / 1000;
prawySilnik = prawySilnik / pulsesPerCm * 20 * 60 * 60 / 100 / 1000;

% tworzenie wykresu
figure('WindowState','maximized');
plot(speedIndexes, lewySilnik, 'o-', 'LineWidth', 3);
hold on;
plot(speedIndexes, prawySilnik, 's-', 'LineWidth', 3);
legend('Prędkość lewego koła, km/h', 'Prędkość prawego koła, km/h', 'Location', 'best');
ylabel('Prędkość rzeczywista, km/h');
xlabel('Prędkość zadana, %');
grid minor;
set(gca,'fontsize', 32)