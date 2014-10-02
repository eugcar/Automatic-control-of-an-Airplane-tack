clear all;
close all;
clc;

disp('Scelta Modalità')
disp(' ')
disp('1: Arduino')
disp('2: Joystick')
disp('3: Automatico')
disp('0: Esci')
disp(' ')

s = input('Inserire Modalità: ');

switch(s)
    case 0
        fprintf('\n Ciao :) \n ');
    case 1
        tesina_aereo_Arduino;
    case 2
        tesina_aereo_joystick;
    case 3
        tesina_aereo_automatico;
end

clear all;
