function [X_sp, w_sp, nPt] = Sigma_points(x, P, w0)
    n = length(x);
    nPt = 2*n+1;
    scale = sqrt(n/(1-w0));
    M = scale*chol(P,'lower');
    X_sp = [-M M zeros(n,1)];
    X_sp = X_sp+repmat(x,1,nPt);
    w_sp = ones(1,nPt)*(1-w0)/(2*n);
    w_sp(nPt) = w0;
end