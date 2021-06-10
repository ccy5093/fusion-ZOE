function [xPts, wPts, nPts] = SigmaPoints_principal_components(x,P,w0)
%Echantillonnage symétrique à 2n+1 sigma points
%PhB janvier 2021
n=length(x);
nPts=2*n+1;
%les sigma points sont mis bout à bout dans une matrice
xPts = zeros(n,2*n+1); %allocation mémoire
%M=chol(P,'lower')%idem 
[V,D] = eig(P);
scale=sqrt(n/(1-w0));
for i=1:n
    xPts(:,i)=x+scale*sqrt(D(i,i))*V(:,i);
    xPts(:,i+n)=x-scale*sqrt(D(i,i))*V(:,i); 
end
xPts(:,nPts)=x;%on rajoute à la fin le point de la moyenne

wPts=ones(1,nPts)*(1-w0)/(2*n);
wPts(nPts)=w0;
