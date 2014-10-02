clear all;
clc;

% INPUT UTENTE %

disp('Controllo Automatico vs Manuale - Aereoplano')
disp(' ')
disp('Cerchio               : Interrompe Simulazione')
disp('Triangolo             : Se premuto passa alla modalità Automatica')
disp('Asse x - Analogico Sx : Controllo Rollio')
disp('Asse x - Analogico Dx : Controllo Imbardata')
disp(' ')

beta_inp = input('Inserire angolo di imbardata (gradi) appartenente all''intervallo [-90:90] :');
if (beta_inp > 0)
    phi_inp = input('Inserire angolo di rollio (gradi) appartenente all''intervallo [0:15] :');
else
    phi_inp = input('Inserire angolo di rollio (gradi) appartenente all''intervallo [-15:0] :');
end
s = input('Inserire tempo di Switching (s) :');

% FASE DI INIZIALIZZAZIONE %

joy = vrjoystick(1);    % Definizione oggetto Joystick

wbeam = 0.5;            % Spessore Linea.
lbeam=100;              % Lunghezza di riferimento assi.
ape_ala = 64.44;        % Apertura Alare Aereo. [m]
lin_imb = 500;          % Usato per la retta da seguire nella sezione Imbardata.
d=2;                    % Fattore di Scala.
ts=0.05;                % Sampling Period [s]
n = s/(2*ts);               % Numero di Iterazioni

phi_rif = degtorad(phi_inp);   % Conversione da Gradi to Radianti di phi
beta_rif = degtorad(beta_inp); % Conversione da Gradi to Radianti di beta

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
    if (button(joy,2) == 1) 
        break;
    end
    
    % Blocco Nbar
    beta_i_n = Nbar(1,1) * beta_rif + Nbar(1,2) * phi_rif;
    phi_i_n = Nbar(2,1) * beta_rif + Nbar(2,2) * phi_rif;
       
    for k=1:n
        
        % Uscita Forzata dal For()
        if (button(joy,2) == 1) 
            break;
        end
        
        pause(ts);

        if(button(joy,1) == 0)

            % Controllo Manuale
            set(1,'name','CONTROLLO MANUALE')

            % Ingressi di controllo Manuale
            dr_m = axis(joy,4)*0.0174*15; 
            da_m = axis(joy,1)*0.0174*25;

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
