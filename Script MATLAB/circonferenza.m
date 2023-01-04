function [p,pd,pdd] = circonferenza(po,r,c,angle,t0,tf,tc,scelta)
%CIRCONFERENZA questa funzione diesgna una circonferenza
%   diamo un punto della circonferenza, l'asse di rotazione e il centro

%pi punto sulla circonferenza
%r asse di rotazione
%c centro
%t0 istante iniziale
%tf istante finale
%scelta indica che tipo di interpolazione fare

%definiamo la matrice di rotazione
xr = (po-c)/norm(po-c);
zr = r;
yr= cross(zr,xr);
R=[xr yr zr]; 

rho = norm(po-c); %raggio

t=t0:tc:tf;
if scelta=="lspb"
    %[s,sd,sdd] = ascissa_curvilinea(angle*rho, t0, tf, t);
    [s,sd,sdd]=lspb(0,angle*rho,t); %profilo di velocità trapezoidale
end

if scelta=="tpoly"
   [s,sd,sdd]=tpoly(0,angle*rho,t); %polinomio quintico
end

%vengono restituiti vettori colonna, ma a noi servono riga
s = s';
sd = sd';
sdd = sdd';

%scriviamo la formula 4.38
ptilde = [rho*cos(s/rho); rho*sin(s/rho); zeros(1,length(s))];
p = c + R*ptilde;
p=p'; %vogliamo sulle righe i timestep e sulle colonne le coordinate x y e z

%calcoliamo velocità e accelerazione
pd = [-sd.*sin(s/rho);
    sd.*cos(s/rho);
    zeros(1,length(sd))
    ];
pd=R*pd;
pd=pd';

pdd = [
    (-sd.^2).*cos(s/rho)/rho-sdd.*sin(s/rho);
    (-sd.^2).*sin(s/rho)/rho+sdd.*cos(s/rho);
    zeros(1,length(sdd))
    ];
pdd=R*pdd;
pdd=pdd';

end

