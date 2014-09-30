%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% N.B. 
%
% Effettua i seguenti collegamenti sulla board Arduino Uno.
%
% Potenziometro degli Ailerons:
%
% VCC -> 2
% INP -> 19
% GND -> 3
%
% Potenziometro del Rudder:
%
% VCC -> 4
% INP -> 18
% GND -> 5
%
% Led:
%
% GND -> 6
% VCC -> 7
% 
% Bottone Stop:
%
% GND -> 8
% INP -> 9
% VCC -> 10
%
% Bottone Switch:
%
% GND -> 11
% INP -> 12
% VCC -> 13
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
clc;

% INPUT UTENTE %

disp('Controllo Automatico vs Manuale - Aereoplano')
disp('Controllo Manuale con -> Arduino ')
disp('Bottone 1               : Interrompe Simulazione')
disp('Bottone 2               : Se premuto passa alla modalità Automatica')
disp('Potenziometro 1 : Controllo Rollio')
disp('Potenziometro 2 : Controllo Imbardata')
disp(' ')

beta_inp = input('Inserire angolo di imbardata (gradi) appartenente all''intervallo [-90:90] :');
if (beta_inp > 0)
    phi_inp = input('Inserire angolo di rollio (gradi) appartenente all''intervallo [0:15] :');
else
    phi_inp = input('Inserire angolo di rollio (gradi) appartenente all''intervallo [-15:0] :');
end
s = input('Inserire tempo di Switching (s) :');

% ARDUINO %

a = arduino('COM3'); % Porta a cui è connessa la scheda Arduino.

a.pinMode(2,'output');
a.pinMode(3,'output');
a.pinMode(4,'output');
a.pinMode(5,'output');
a.pinMode(6,'output');
a.pinMode(7,'output');
a.pinMode(8,'output');
a.pinMode(10,'output');
a.pinMode(11,'output');
a.pinMode(13,'output');

a.pinMode(9,'input');   % Input -> Bottone_stop
a.pinMode(12,'input');  % Input -> Bottone_switch
a.pinMode(18,'input');  % Input -> Potenziomentro_Imbardata
a.pinMode(19,'input');  % Input -> Potenziomentro_Rollio

a.digitalWrite(2,1);  % VCC : Potenziometro_rollio
a.digitalWrite(3,0);  % GND : Potenziometro_rollio
a.digitalWrite(4,1);  % VCC : Potenziometro_imbardata
a.digitalWrite(5,0);  % GND : Potenziometro_imbardata

a.digitalWrite(6,0);  % GND : Led
a.digitalWrite(7,0);  % VCC : Led -> Settato nullo in quanto con il
                      % controllo manuale il led è spento.
a.digitalWrite(8,0);  % GND : Bottone_Stop
a.digitalWrite(10,1); % VCC : Bottone_Stop
a.digitalWrite(11,0); % GND : Bottone_Switch
a.digitalWrite(13,1); % VCC : Bottone_Switch

% FASE DI INIZIALIZZAZIONE %

wbeam = 0.5;                    % Spessore Linea. [m]
lbeam=100;                      % Lunghezza di riferimento assi. [m]
ape_ala = 64.44;                % Apertura Alare Aereo. [m]
lin_imb = 500;                  % Usato per la retta da seguire nella sezione Imbardata. [m]
d=2;                            % Fattore di Scala.
ts=0.05;                        % Sampling Period [s]
n = s/(2*ts);                   % Numero di Iterazioni
rudder_max = degtorad(15);      % Angolo massimo Rudder [rad]
ailerons_max = degtorad(25);    % Angolo massimo Ailerons [rad]

% Questi valori sono quelli letti dai potenziometri, se si discostano
% di molto da quelli misurati, modificarli.

pot_rud_min = 24;
pot_rud_max = 868;
pot_ail_min = 24;
pot_ail_max = 868;

% Vettori per il mapping da potenziometro -> ingresso di controllo.

map_rudder = mapping_angle_arduino(pot_rud_min,pot_rud_max,-rudder_max,rudder_max);
map_ailerons = mapping_angle_arduino(pot_ail_min,pot_ail_max,-ailerons_max,ailerons_max);

phi_rif = degtorad(phi_inp);    
beta_rif = degtorad(beta_inp);  

% DEFINIZIONE DEL SISTEMA %

A = [-0.0558 -0.9968  0.0802  0.0415; 
      0.598   -0.115   -0.0318 0; 
      -3.05    0.388   -0.465  0; 
      0        0.0805  1       0];
B = [0.0729 -4.75 1.53 0; 0.0001 1.23 10.63 0]';
C = eye(4); % y = x

sysC = ss(A, B, C, 0);
sysD = c2d(sysC, ts, 'zoh');   % Sistema Discretizzato
[PHI, GAM] = ssdata(sysD);     % Estraiamo le matrici dal sistema discreto

Cd = [1 0 0 0; 0 0 0 1];       % Matrice di uscita -> simulazione dei sensori.

% Controllore / Osservatore
K = place(PHI, GAM, [.965 .970 .975 .980]);    % Usiamo place() piuttosto che acker() in quanto 
O = place(PHI', PHI'*Cd',[.30 .35 .40 .45])';  % trattiamo un sistema MIMO.

% Selezione del riferimento
Hr = [ 1 0 0 0 ; 0 0 0 1 ]; 

% Nx ,Nu ,Nbar
[Nx, Nu, Nbar] = refi(PHI, GAM, Hr, K);

% Preallochiamo lo spazio necessario alle iterazioni per aumentarne la
% velocità.

beta = zeros(1,n);
r = zeros(1,n);
p = zeros(1,n);
phi = zeros(1,n);

beta_oss = zeros(1,n);
r_oss = zeros(1,n);
p_oss = zeros(1,n);
phi_oss = zeros(1,n);

beta_k = zeros(1,n);
phi_k = zeros(1,n);

dr = zeros(1,n);
da = zeros(1,n);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARTE GRAFICA %

figure(1)
set(1,'NumberTitle','off')
% Sezione Rollio %

subplot(1,2,1);
set(1,'Position',[20 180 1324 470]) % Imposta grandezza della finestra.
axis([-0.6*lbeam 0.6*lbeam -0.2*lbeam 0.2*lbeam]); % Creiamo il sistema di riferimento.
axis equal

hold on 

% Rappresentazione Aereo Posteriore
xdbeam=[ape_ala/d;ape_ala/d;-ape_ala/d;-ape_ala/d];
ydbeam=[-wbeam/d;wbeam/d;wbeam/d;-wbeam/d];
pbeam=fill(xdbeam,ydbeam,'black','EraseMode','background');
xlabel('Vista Posteriore Aereoplano','FontSize',16)

hold on

% Linea di Equilibrio Rollio
xlineEq_r=[-2*ape_ala*cos(phi_rif);2*ape_ala*cos(phi_rif)]; 
ylineEq_r=[-2*ape_ala*sin(phi_rif);2*ape_ala*sin(phi_rif)];
pline_r = plot(xlineEq_r,ylineEq_r,'green');

% Sezione Imbardata %

subplot(1,2,2);

axis([-3*lbeam 3*lbeam -3*lbeam 3*lbeam]); % Creiamo il sistema di riferimento.
axis equal 

hold on 

% Rappresentazione Aereo Superiore
xdtriangle=[-ape_ala/2;0;ape_ala/2];
ydtriangle=[-ape_ala/2;ape_ala;-ape_ala/2];
ptriangle=fill(xdtriangle,ydtriangle,'black','EraseMode','background');  
xlabel('Vista Superiore Aereoplano','FontSize',16)

hold on

% Linea di Equilibrio Imbardata
xlineEq_i=[lin_imb*sin(beta_rif);-lin_imb*sin(beta_rif)]; 
ylineEq_i=[-lin_imb*cos(beta_rif);lin_imb*cos(beta_rif)];
pline_i = plot(xlineEq_i,ylineEq_i,'green');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while(1)
    
    % Uscita Forzata dal While()
    if (a.digitalRead(9) == 1) 
        break;
    end
    
    % Blocco Nbar
    beta_i_n = Nbar(1,1) * beta_rif + Nbar(1,2) * phi_rif;
    phi_i_n = Nbar(2,1) * beta_rif + Nbar(2,2) * phi_rif;
       
    for k=1:n
        
        % Uscita Forzata dal For()
        if (a.digitalRead(9) == 1) 
            break;
        end
        
        pause(ts);

        if(a.digitalRead(12) == 0)

            % Controllo Manuale
            set(1,'name','CONTROLLO MANUALE') % Modifico Titolo Figure
            a.digitalWrite(7,0);                % Spegnimento del Led

            % Ingressi di controllo Manuale
            pot_imb = a.analogRead(4);  % Lettura Potenziometro_Imbardata
            pot_rol = a.analogRead(5);  % Lettura Potenziometro_Rollio
            
            % Mapping Potenziometro -> Controllo
            dr_m = map_rudder(pot_imb); 
            da_m = map_ailerons(pot_rol);

            % Plant Discretizzato Manuale
            beta(k+1) = PHI(1,1)*beta(k) + PHI(1,2)*r(k) + PHI(1,3)*p(k) + PHI(1,4)*phi(k) + GAM(1,1)*dr_m + GAM(1,2)*da_m;
            r(k+1)    = PHI(2,1)*beta(k) + PHI(2,2)*r(k) + PHI(2,3)*p(k) + PHI(2,4)*phi(k) + GAM(2,1)*dr_m + GAM(2,2)*da_m;
            p(k+1)    = PHI(3,1)*beta(k) + PHI(3,2)*r(k) + PHI(3,3)*p(k) + PHI(3,4)*phi(k) + GAM(3,1)*dr_m + GAM(3,2)*da_m;
            phi(k+1)  = PHI(4,1)*beta(k) + PHI(4,2)*r(k) + PHI(4,3)*p(k) + PHI(4,4)*phi(k) + GAM(4,1)*dr_m + GAM(4,2)*da_m;

            phi_o = phi(k+1);   % Usato per uniformare in output ctrl Man / Auto
            beta_o = beta(k+1); % Usato per uniformare in output ctrl Man / Auto

            da_o = da_m; % Usato per uniformare in output ctrl Man / Auto
            dr_o = dr_m; % Usato per uniformare in output ctrl Man / Auto

        else

            % Controllo Automatico
            
            set(1,'name','CONTROLLO AUTOMATICO')
            a.digitalWrite(7,1);                % Accensione del Led

            % Plant Discretizzato Automatico
            beta(k+1) = PHI(1,1)*beta(k) + PHI(1,2)*r(k) + PHI(1,3)*p(k) + PHI(1,4)*phi(k) + GAM(1,1)*dr(k) + GAM(1,2)*da(k);
            r(k+1)    = PHI(2,1)*beta(k) + PHI(2,2)*r(k) + PHI(2,3)*p(k) + PHI(2,4)*phi(k) + GAM(2,1)*dr(k) + GAM(2,2)*da(k);
            p(k+1)    = PHI(3,1)*beta(k) + PHI(3,2)*r(k) + PHI(3,3)*p(k) + PHI(3,4)*phi(k) + GAM(3,1)*dr(k) + GAM(3,2)*da(k);
            phi(k+1)  = PHI(4,1)*beta(k) + PHI(4,2)*r(k) + PHI(4,3)*p(k) + PHI(4,4)*phi(k) + GAM(4,1)*dr(k) + GAM(4,2)*da(k);

            % Blocco Hr
            beta_o = beta(k+1);
            phi_o = phi(k+1);

            % Blocco Cd
            beta_cd = beta(k+1);
            phi_cd = phi(k+1);

            % Osservatore
            beta_oss(k+1) = PHI(1,1)*beta_oss(k) + PHI(1,2)*r_oss(k) + PHI(1,3)*p_oss(k) + PHI(1,4)*phi_oss(k) + GAM(1,1)*dr(k) + GAM(1,2)*da(k) + O(1,1)*(beta_cd - (Cd(1,1)*beta_oss(k) + Cd(1,2)*r_oss(k) + Cd(1,3)*p_oss(k) + Cd(1,4)*phi_oss(k))) + O(1,2)*(phi_cd - (Cd(2,1)*beta_oss(k) + Cd(2,2)*r_oss(k) + Cd(2,3)*p_oss(k) + Cd(2,4)*phi_oss(k)));
            r_oss(k+1)    = PHI(2,1)*beta_oss(k) + PHI(2,2)*r_oss(k) + PHI(2,3)*p_oss(k) + PHI(2,4)*phi_oss(k) + GAM(2,1)*dr(k) + GAM(2,2)*da(k) + O(2,1)*(beta_cd - (Cd(1,1)*beta_oss(k) + Cd(1,2)*r_oss(k) + Cd(1,3)*p_oss(k) + Cd(1,4)*phi_oss(k))) + O(2,2)*(phi_cd - (Cd(2,1)*beta_oss(k) + Cd(2,2)*r_oss(k) + Cd(2,3)*p_oss(k) + Cd(2,4)*phi_oss(k)));
            p_oss(k+1)    = PHI(3,1)*beta_oss(k) + PHI(3,2)*r_oss(k) + PHI(3,3)*p_oss(k) + PHI(3,4)*phi_oss(k) + GAM(3,1)*dr(k) + GAM(3,2)*da(k) + O(3,1)*(beta_cd - (Cd(1,1)*beta_oss(k) + Cd(1,2)*r_oss(k) + Cd(1,3)*p_oss(k) + Cd(1,4)*phi_oss(k))) + O(3,2)*(phi_cd - (Cd(2,1)*beta_oss(k) + Cd(2,2)*r_oss(k) + Cd(2,3)*p_oss(k) + Cd(2,4)*phi_oss(k)));
            phi_oss(k+1)  = PHI(4,1)*beta_oss(k) + PHI(4,2)*r_oss(k) + PHI(4,3)*p_oss(k) + PHI(4,4)*phi_oss(k) + GAM(4,1)*dr(k) + GAM(4,2)*da(k) + O(4,1)*(beta_cd - (Cd(1,1)*beta_oss(k) + Cd(1,2)*r_oss(k) + Cd(1,3)*p_oss(k) + Cd(1,4)*phi_oss(k))) + O(4,2)*(phi_cd - (Cd(2,1)*beta_oss(k) + Cd(2,2)*r_oss(k) + Cd(2,3)*p_oss(k) + Cd(2,4)*phi_oss(k)));

            % Blocco K
            beta_k(k+1) = K(1,1)*beta_oss(k+1) + K(1,2)*r_oss(k+1) + K(1,3)*p_oss(k+1) + K(1,4)*phi_oss(k+1);
            phi_k(k+1) = K(2,1)*beta_oss(k+1) + K(2,2)*r_oss(k+1) + K(2,3)*p_oss(k+1) + K(2,4)*phi_oss(k+1);

            % Input Plant (errore)
            dr(k+1) = beta_i_n - beta_k(k+1);
            da(k+1) = phi_i_n - phi_k(k+1);

            da_o = da(k+1); % Usato per uniformare in output ctrl Man / Auto
            dr_o = dr(k+1); % Usato per uniformare in output ctrl Man / Auto

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % AGGIORNAMENTO PARTE GRAFICA %

        % Cambio coordinate rollio
        ur=[cos(phi_o) -sin(phi_o); sin(phi_o) cos(phi_o)]; % Matrice Cambio Coordinate Rollio
        dbeam_r=[xdbeam ydbeam]*ur';
        xpbeam_r=dbeam_r(:,1); ypbeam_r=dbeam_r(:,2);
        set(pbeam,'Xdata',xpbeam_r,'Ydata',ypbeam_r); % Modifichiamo i valori di (x.y) in funzione dell'angolo phi.

        subplot(1,2,1)
        phi_deg = radtodeg(phi_o);
        da_deg = radtodeg(da_o);
        title(['Angolo di Rollio: ' num2str(phi_deg)]);     % Stampo Angolo di Rollio
        ylabel(['Angolazione Ailerons: ' num2str(da_deg)]); % Stampo Angolo degli Ailerons

        % Cambio coordinate imbardata
        ui=[cos(beta_o) -sin(beta_o); sin(beta_o) cos(beta_o)]; % Matrice Cambio Coordinate Imbardata
        dbeam_i=[xdtriangle ydtriangle]*ui';
        xpbeam_i=dbeam_i(:,1); ypbeam_i=dbeam_i(:,2);
        set(ptriangle,'Xdata',xpbeam_i,'Ydata',ypbeam_i); % Modifichiamo i valori di (x.y) in funzione dell'angolo beta.

        subplot(1,2,2)
        beta_deg = radtodeg(beta_o);
        dr_deg = radtodeg(dr_o);
        title(['Angolo di Imbardata: ' num2str(beta_deg)]); % Stampo Angolo di Imbardata
        ylabel(['Angolazione Rudder: ' num2str(dr_deg)]);   % Stampo Angolo del Rudder

        drawnow;

    end
    
    % Switching del riferimento.
    phi_rif = -phi_rif;
    beta_rif = -beta_rif;
    
    % Switching Linee di Equilibrio
    
    subplot(1,2,1)
    % Linea di Equilibrio Rollio
    xlineEq_r=[-2*ape_ala*cos(phi_rif);2*ape_ala*cos(phi_rif)]; 
    ylineEq_r=[-2*ape_ala*sin(phi_rif);2*ape_ala*sin(phi_rif)];
    set(pline_r,'Xdata',xlineEq_r,'Ydata',ylineEq_r);
    
    subplot(1,2,2)
    % Linea di Equilibrio Imbardata
    xlineEq_i=[lin_imb*sin(beta_rif);-lin_imb*sin(beta_rif)]; 
    ylineEq_i=[-lin_imb*cos(beta_rif);lin_imb*cos(beta_rif)];
    set(pline_i,'Xdata',xlineEq_i,'Ydata',ylineEq_i);
        
    % Continuazione dello Stato
    
    beta(1) = beta(k+1);
    r(1) = r(k+1);
    p(1) = p(k+1);
    phi(1) = phi(k+1);
    
end

close all;
index

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NOTE %

% Velocità di volo : 0.8 Mach
% Altezza di volo : 40000 ft

% beta = angolo di imbardata
% r = velocità angolare di imbardata
% p = velocità angolare di rollio
% phi = angolo di rollio

% da = angolo dell'Ailerons
% dr = angolo del Rudder