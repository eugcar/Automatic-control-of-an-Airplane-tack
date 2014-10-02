% In questa funzione prendiamo in input un range di valori POSITIVI, e 
% li andiamo a mappare in un certo range, ad esempio :
%
% Un potenziometro ci fornisce valori da 10 a 890, noi vogliamo mapparli
% da -30 a 30, in questo caso avremo:
%
% inp_min = 10;
% inp_max = 890;
% out_min = -30;
% out_max = 30;
%
% Il nostro output sarà un array i cui valori saranno -30 fino all'elemento
% in posizione 9, dal 10 al 890, avremo tutti i valori da -30 a 30, con
% incremento pari a step e gli ultimi 10 valori saranno 30.

function [map] = mapping_angle_arduino(inp_min, inp_max, out_min, out_max)

% Numero di valori nell'intervallo di input
n = inp_max - inp_min; 
% Step di incremento
step = (abs(out_max) + abs(out_min))/n; 

% map_ini e map_fin servono a garantire un minimo di oscillazione del
% valore di input nel caso minimo e nel caso massimo.
map_ini = out_min*ones(1,(inp_min -1));
map_fin = out_max*ones(1,10);

% Completiamo l'array di mapping inp - out.
map = [map_ini out_min:step:out_max map_fin];



