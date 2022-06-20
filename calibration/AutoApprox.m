%% 
clc
clear
close all
%%
FT = load('T.txt'); % загрузка файла с записью результатов полёта
MinTemp = min(FT(:, 8));
IndexFirst = min(FT(:, 1));
IndexLast = max(FT(:, 1));
StartTemp = MinTemp;
IndexCount = IndexLast - IndexFirst; % кол-во строк в записи

TempStep = 2; % температурный шаг

%% Разбиение данных на участки по индексам через шаг по температуре
IndexPointsVector(1) = IndexFirst;
LinesNumber = 1;
for Index = IndexFirst : 1 : (IndexLast - IndexFirst)
    if FT(Index, 8) > (StartTemp + TempStep) 
        StartTemp = StartTemp + TempStep;
        LinesNumber = LinesNumber + 1; %вычисление кол-ва отрезков, на которые разбивается запись
        IndexPointsVector = [IndexPointsVector Index];
    end
end

IndexPointsVector = [IndexPointsVector (IndexLast - IndexFirst)];
LinesNumber = LinesNumber + 1;

%% Вектор температур, соответствующих точкам разбиения по индексу 

TempPointsVector = double.empty(0, LinesNumber);
for i = 1 : 1 : length(IndexPointsVector)
    TempPointsVector(i) = FT(IndexPointsVector(i), 8);
end

%% Матрица с узлами

NodesArray = TempPointsVector(:);
for Column = 2 : 1 : 7
    YPointsVector = TruePointsFinder(LinesNumber, FT, IndexPointsVector, Column);
    YPointsVector = YPointsVector(:);
    NodesArray = [NodesArray YPointsVector];
end

YPointsVector = TruePointsFinder(LinesNumber, FT, IndexPointsVector, Column);

% Points = [TempPointsArray; A];

MultiPlot(FT, Column, IndexPointsVector, TempPointsVector, YPointsVector)

fileID = fopen('Approx.txt','w');
writematrix(NodesArray,'Approx.txt','Delimiter','tab')

%% Получение узлов

function YPointsArray = TruePointsFinder(Num, F, POINTS, Column)

for i = 1 : 1 : (Num-1)
    x = POINTS(i) : 1 : POINTS(i+1);
    t = spline(F(:, 1), F(:, 8), x);
    
    ydef = spline(F(:, 1), F(:, Column), x);
    
    P = polyfit(t, ydef, 1);

    yapp = (P(1) * t + P(2));
    
    if i == 1
        YPointsArray(1) = (P(1) * F(POINTS(i),8) + P(2));
        ycurrent = (P(1) * F(POINTS(i+1),8) + P(2));
    end
    if i > 1
        YPointsArray = [YPointsArray (ycurrent + (P(1) * F(POINTS(i),8) + P(2)))/2];
        ycurrent = (P(1) * F(POINTS(i),8) + P(2));
    end
    if i == (Num-1)
        YPointsArray = [YPointsArray (P(1) * F(POINTS(Num-1),8) + P(2))];
    end
end
end

%% Построение графика
function MultiPlot(F, Col, IndexPoints, Temp, Y)
for i = 1 : 1 : (size(F)-1)

    x = IndexPoints(i) : 1 : IndexPoints(i+1);
    t = spline(F(:, 1), F(:, 8), x);
    
    y = spline(F(:, 1), F(:, Col), x);
    xlabel('сигнал датчика температуры, град')
    ylabel('сигнал датчика угловой скорости, град/c')
    plot(t, y, 'Color',[0.25,0.36,0.91])
    hold on
    grid on
    line(Temp, Y, 'Color', [0,0,0])
    
end
end
