clc
clear
close all

%% Temp Correction applyment


%% Загрузка  файлов
Pas = load('passport.txt');

P1 = load('pos1.txt'); % загрузка файла с записью результатов полёта    Pos1
P2 = load('pos2.txt'); % загрузка файла с записью результатов полёта    Pos2
P3 = load('pos3.txt'); % загрузка файла с записью результатов полёта    Pos3
P4 = load('pos4.txt'); % загрузка файла с записью результатов полёта    Pos4
P5 = load('pos5.txt'); % загрузка файла с записью результатов полёта    Pos5
P6 = load('pos6.txt'); % загрузка файла с записью результатов полёта    Pos6

Fx1 = load('HrXrep2.txt'); % загрузка файла с записью результатов полёта    Rot1
Fy1 = load('HrYrep2.txt'); % загрузка файла с записью результатов полёта    Rot2
Fz1 = load('HrZrep2.txt'); % загрузка файла с записью результатов полёта    Rot3

%% константы
g_m_s_s = 9.80665; % ускорение свободного падения м/с^2
U_rad_s = 2*3.14/86400; % угловая скорость вращения Земли

OM_XYZ = [1 2 3];
A_XYZ = [4 5 6];

%% вычисление среднийх значений по столбцам

PF1 = FixValues(Pas, P1);
PF2 = FixValues(Pas, P2);
PF3 = FixValues(Pas, P3);
PF4 = FixValues(Pas, P4);
PF5 = FixValues(Pas, P5);
PF6 = FixValues(Pas, P6);

FFx1 = FixValues(Pas, Fx1);
FFy1 = FixValues(Pas, Fy1);
FFz1 = FixValues(Pas, Fz1);

IndexF_FFx1 = 1;
IndexL_FFx1 = size(FFx1);

IndexF_FFy1 = 1;
IndexL_FFy1 = size(FFy1);

IndexF_FFz1 = 1;
IndexL_FFz1 = size(FFz1);

%% Alpha matrix

A = [Filter(PF4,A_XYZ(1)) - Filter(PF2, A_XYZ(1)) Filter(PF5,A_XYZ(1)) - Filter(PF6, A_XYZ(1)) Filter(PF1,A_XYZ(1)) - Filter(PF3, A_XYZ(1));
     Filter(PF4,A_XYZ(2)) - Filter(PF2, A_XYZ(2)) Filter(PF5,A_XYZ(2)) - Filter(PF6, A_XYZ(2)) Filter(PF1,A_XYZ(2)) - Filter(PF3, A_XYZ(2));
     Filter(PF4,A_XYZ(3)) - Filter(PF2, A_XYZ(3)) Filter(PF5,A_XYZ(3)) - Filter(PF6, A_XYZ(3)) Filter(PF1,A_XYZ(3)) - Filter(PF3, A_XYZ(3))];

A = inv(A) * [2*g_m_s_s 0 0; 0 2*g_m_s_s 0; 0 0 2*g_m_s_s]; 

%% Alpha vector

alpha = 1/6 * A * [Filter(PF1, A_XYZ(1)) + Filter(PF2, A_XYZ(1)) + Filter(PF3, A_XYZ(1)) + Filter(PF4, A_XYZ(1)) + Filter(PF5, A_XYZ(1)) + Filter(PF6, A_XYZ(1));
                   Filter(PF1, A_XYZ(2)) + Filter(PF2, A_XYZ(2)) + Filter(PF3, A_XYZ(2)) + Filter(PF4, A_XYZ(2)) + Filter(PF5, A_XYZ(2)) + Filter(PF6, A_XYZ(2));
                   Filter(PF1, A_XYZ(3)) + Filter(PF2, A_XYZ(3)) + Filter(PF3, A_XYZ(3)) + Filter(PF4, A_XYZ(3)) + Filter(PF5, A_XYZ(3)) + Filter(PF6, A_XYZ(3))];

%% Beta matrix

B = [0 0 0; 0 0 0; 0 0 0];

delta = 0.01 ; % дельта, сек

for Index = IndexF_FFx1 : 1 : (IndexL_FFx1(1) - IndexF_FFx1)
    B(1,1) = B(1,1) + FFx1(Index, 2) * delta;
    B(2,1) = B(2,1) + FFx1(Index, 3) * delta;
    B(3,1) = B(3,1) + FFx1(Index, 4) * delta;
end

for Index = IndexF_FFy1 : 1 : (IndexL_FFy1(1) - IndexF_FFy1)
    B(1,2) = B(1,2) + FFy1(Index, 2) * delta;
    B(2,2) = B(2,2) + FFy1(Index, 3) * delta;
    B(3,2) = B(3,2) + FFy1(Index, 4) * delta;
end

for Index = IndexF_FFz1 : 1 : (IndexL_FFz1(1) - IndexF_FFz1)
    B(1,3) = B(1,3) + FFz1(Index, 2) * delta;
    B(2,3) = B(2,3) + FFz1(Index, 3) * delta;
    B(3,3) = B(3,3) + FFz1(Index, 4) * delta;
end

B = inv(B) * [180 0 0; 0 180 0; 0 0 180];

%% AB

AB = (1/g_m_s_s * (B * [(Filter(PF4,OM_XYZ(1)) - Filter(PF2, OM_XYZ(1)))/2; (Filter(PF4,OM_XYZ(2)) - Filter(PF2, OM_XYZ(2)))/2; (Filter(PF4,OM_XYZ(3)) - Filter(PF2, OM_XYZ(3)))/2]));
AB = [AB 1/g_m_s_s * (B * [(Filter(PF5,OM_XYZ(1)) - Filter(PF6, OM_XYZ(1)))/2; (Filter(PF5,OM_XYZ(2)) - Filter(PF6, OM_XYZ(2)))/2; (Filter(PF5,OM_XYZ(3)) - Filter(PF6, OM_XYZ(3)))/2])];
AB = [AB 1/g_m_s_s * (B * [(Filter(PF1,OM_XYZ(1)) - Filter(PF3, OM_XYZ(1)))/2; (Filter(PF1,OM_XYZ(2)) - Filter(PF3, OM_XYZ(2)))/2; (Filter(PF1,OM_XYZ(3)) - Filter(PF3, OM_XYZ(3)))/2])];

%% Beta vector

beta = B * [(Filter(PF5,OM_XYZ(1)) + Filter(PF6, OM_XYZ(1)))/2; (Filter(PF5,OM_XYZ(2)) + Filter(PF6, OM_XYZ(2)))/2; (Filter(PF5,OM_XYZ(3)) + Filter(PF6, OM_XYZ(3)))/2];


fileID = fopen('Сpasport.txt','a');
precision = 8;
dlmwrite('Сpasport.txt',Pas,'delimiter','\t','precision',precision)
dlmwrite('Сpasport.txt',A,'delimiter','\t','precision',precision, '-append','roffset',1)
dlmwrite('Сpasport.txt',alpha,'delimiter','\t','precision',precision, '-append','roffset',1)
dlmwrite('Сpasport.txt',B,'delimiter','\t','precision',precision, '-append','roffset',1)
dlmwrite('Сpasport.txt',beta,'delimiter','\t','precision',precision, '-append','roffset',1)
dlmwrite('Сpasport.txt',AB,'delimiter','\t','precision',precision, '-append','roffset',1)

fclose(fileID);

CP = fopen('calp.dat','w');
fwrite(CP,Pas);
fwrite(CP,A);
fwrite(CP,alpha);
fwrite(CP,B);
fwrite(CP,beta);
fwrite(CP,AB);

fclose(CP);

%% Filter

function F = Filter(P, N)

IndexFirst = 1;
IndexLast = size(P);
F = P(IndexFirst,N);
K = 2;

for I = (IndexFirst + 1) : 1 : (IndexLast - IndexFirst)
    F = (F *(K-1) + P(I,N))/K;
    K = K + 1;
end
end

function V = FixValues(Pasport, File) % создание массива значений, скорректированных по температурной калибровке
V = double.empty;
        
for Str = 1 : 1 : size(File)
    P = double.empty;
    Y = double.empty;
    g_m_s_s = 9.80665;
    F = 0;
    for i = 1 : 1 : size(Pasport)
        if F == 0
            if File(Str,8) ~= 0.00 && Pasport(i,1) > File(Str,8)
                F = 1;
                for k = 2 : 1 : 7
                    P = [P [((Pasport(i,k)-Pasport(i-1,k))/(Pasport(i,1)-Pasport(i-1,1))); (Pasport(i-1,k) - (Pasport(i,k)-Pasport(i-1,k))/(Pasport(i,1)-Pasport(i-1,1))*Pasport(i-1,1))]];
                end
            end
        end
    end %поиск температурного промежутка
    if F ~= 0
        for j = 2 : 1 : 7
            if j == 7
                Y = [Y (File(Str, j) - (P(1, j-1)*File(Str, 8) + P(2, j-1)- g_m_s_s))]; 
            else
                Y = [Y (File(Str, j) - (P(1, j-1)*File(Str, 8) + P(2, j-1)))];
            end
        end %коррекция значений с помощью коэффициентов
        V = [V; Y];
    end
end
end


