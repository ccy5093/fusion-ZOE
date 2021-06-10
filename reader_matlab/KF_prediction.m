function [x,P] = KF_prediction(x1, P1, Q, A, u, B)
    if nargin>4         % If has input
        x = A*x1+B*u;
        P = A*P1*A'+Q;
    else                % No input case
        x = A*x1;
        P = A*P1*A'+Q;
    end
end

