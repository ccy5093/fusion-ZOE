function [x,P,omega]=CI_update(x1,P1,x2,P2,C,omega)
    %CI data fusion of two estimates or a prediction with a measurement
    %If omega (0<omega<1)is omitted then the routine computes the ellipsoid 
    %that has the minimum volume
    %C : observation matrix or Jacobian of the observation model
    P1i=inv(P1);
    P2i=inv(P2);

    if nargin < 6 %compute omega
        f=inline('1/det(omega*P1i+(1-omega)*C*P2i*C)','omega', 'P1i', 'P2i', 'C');
        %Work out omega using the matlab constrained minimizer function
        %omega=fminbnd(f,0,1,optimset('Display','off'),P1i,P2i,C);
        %The unconstrained version of this optimization is
        omega=fminsearch(f,0.5,optimset('Display','off'),P1i,P2i,C);
        omega=min(max(omega,0),1); %saturation
    end
    P = inv(omega*P1i+(1-omega)*C'*P2i*C);
    x = x1 + (1-omega)*P*C'*P2i*(x2-C*x1);
end