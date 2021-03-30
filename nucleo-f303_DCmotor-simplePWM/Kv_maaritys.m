% DC-moottorin Kv-arvon muodostus
% Mitatut arvot vektoreissa Ug ja f

% Pyöritysmoottorin napajännite (ei käytetä mihinkään tässä)
Us = [ 4 5 6 7 8 9 10 ]

% Mitattavan moottorin napajännite (generaattorijännite)
Ug = [ 1.92 2.78 3.66 4.54 5.45 6.38 7.28 9.1 ]

% Pyörimisanturilta saatu taajuus Hz (1 jakso per kierros)
f = [ 34 50.2 66.2 81.3 98.5 116 132 167 ]

% Hertsit radiaaneiksi sekunnissa
w = f*2*pi

% Hertsit RPM:ksi näyttöön
RPM = f*60

% Lasketaan Kv -vektori
Kv = w ./ Ug

% Plotataan generaattorijännite ja vastaava Kv-arvo kierrosluvun funktiona
[p, y1, y2] = plotyy( RPM, Ug, RPM, Kv )
xlabel (p(1), "RPM", 'FontSize', 24);
ylabel (p(1), "Ugen", 'FontSize', 24);
ylabel (p(2), "Kv", 'FontSize', 24);
set(p,'FontSize', 18, 'LineWidth', 1);
set(y1, 'LineWidth', 3);
set(y2, 'LineWidth', 3);
grid on;
grid minor on;
Kv_avg = mean(Kv)
Kt = 1/Kv_avg