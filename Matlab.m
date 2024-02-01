clc
clear all
close all

% Inicjalizacja zmiennych
filePrefix = 'Data_';  % Prefiks nazwy pliku
fileExtension = '.csv';  % Rozszerzenie pliku
startRow = 5;  % Numer pierwszego wiersza do odczytu
endRow = 24;  % Numer ostatniego wiersza do odczytu
averageRows = 10:24;  % Zakres wierszy do obliczenia średniej

% Inicjalizacja wektora numerów plików
fileNumbers = 0:5:100;

% Inicjalizacja wektorów do przechowywania średnich wartości
averageColumn1 = zeros(size(fileNumbers));
averageColumn2 = zeros(size(fileNumbers));

% Obliczenie średnich wartości dla każdego pliku
for idx = 1:length(fileNumbers)
    x = fileNumbers(idx);
    
    % Generowanie nazwy pliku
    fileName = [filePrefix, num2str(x), fileExtension];
    
    % Odczyt danych z pliku
    data = abs(readmatrix(fileName));
    
    % Obliczanie średnich wartości dla kolumny 1 i 2
    averageColumn1(idx) = mean(data(averageRows, 1));
    averageColumn2(idx) = mean(data(averageRows, 2));
end

% Tworzenie wykresu dla kolumny 1
figure('WindowState','maximized');
plot(fileNumbers, averageColumn1, 'o-', 'LineWidth', 2);
ylabel('Pulsy lewego silnika na pętlę, -');
xlabel('Prędkość, %');
grid minor;

% Tworzenie wykresu dla kolumny 2
figure('WindowState','maximized');
plot(fileNumbers, averageColumn2, 'o-', 'LineWidth', 2);
ylabel('Pulsy prawego silnika na pętlę, -');
xlabel('Prędkość, %');
grid minor;