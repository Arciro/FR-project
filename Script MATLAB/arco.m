function [p,pd,pdd] = arco(p0,pf,r,rho,t0,tf,tc,verso,scelta)
%UNTITLED Questa funzione diesgna un arco di circonferenza
% diamo punti iniziali e finali, l'asse di rotazione, il raggio

%p0 punto iniziale
%pf punto finale
%r asse di rotazione
%rho raggio
%t0 istante iniziale
%tf instante finale
%verso indice se l'arco deve essere orientato in senso orario o antiorario
%scelta indica come deve essere effettuata l'interpolazione

syms x y z real

d=pf-p0; %distanza tra i due punti
pm = [(p0(1)+pf(1))/2; (p0(2)+pf(2))/2; (p0(3)+pf(3))/2]; %punto medio

%se il raggio è più piccolo del metà della distanza tra punto iniziale
%e finale allora settalo automaticamente al valore pari a tale distanza
if rho <= norm(pf-p0)/2
   rho = norm(pf-p0)/2+0.00001;
end

%equazioni da risolvere a sistema
piano1=d(1)*(x-pm(1))+d(2)*(y-pm(2))+d(3)*(z-pm(3));
piano2=r(1)*(x-p0(1))+r(2)*(y-p0(2))+r(3)*(z-p0(3));
sfera=(x-p0(1))^2+(y-p0(2))^2+(z-p0(3))^2-rho^2;

%coordinate del centro
[cx,cy,cz] = solve([piano1, piano2, sfera], [x, y, z]);
cx = double(cx);
cy = double(cy);
cz = double(cz);

% dobbiamo decidere quale dei 2 centri ottenuti considerare, solo che
% se il raggio è minore della metà della distanza tra i due punti allora
% centro è il punto medio tra punto iniziale e finale e ce ne sarà solo uno
if rho <= norm(pf-p0)/2
    centro = [cx; cy; cz];
    
else
   if verso=="orario"
      centro = [cx(1); cy(1); cz(1)];
   end

   if verso=="antiorario"
      centro = [cx(2); cy(2); cz(2)];
   end 
end

%definiamo la matrice di rotazione;
xr = (p0-centro)/norm(p0-centro);
zr = r;
yr= cross(zr,xr);
R=[xr yr zr];

%ora andiamoci a calcolare i punti ptilde corrispondenti a p0 e pf
ptilde0 = R'*(p0-centro);
ptildef = R'*(pf-centro);
s0 = rho*atan2(ptilde0(2), ptilde0(1));
sf = rho*atan2(ptildef(2), ptildef(1));

%definiamo il vettore dei tempi e interpoliamo i valori iniziali e finali
% di ascissa curvilienea con polinomio quintico o con cubico il quale ha
%un profilo di velocità trapezoidale
t=t0:tc:tf;
if scelta=="lspb"
   [s,sd,sdd]=lspb(s0,sf,t); %profilo di velocità trapezoidale
end

if scelta=="tpoly"
   [s,sd,sdd]=tpoly(s0,sf,t); %polinomio quintico
end

%vengono restituiti vettori colonna, ma a noi servono riga
s = s';
sd = sd';
sdd = sdd';

%scriviamo la formula 4.38
ptilde = [rho*cos(s/rho); rho*sin(s/rho); zeros(1,length(s))];
p = centro + R*ptilde;
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

