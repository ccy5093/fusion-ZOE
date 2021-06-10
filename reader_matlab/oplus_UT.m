function [pos_out, cov_out] = oplus_UT(pos_a, pos_b, cov_a, cov_b,w0)
    if nargin<5
        w0 = 0.2;
    end
    [X_sp_a,w_sp_a,nPt_a]=Sigma_points(pos_a,cov_a,w0);
    [X_sp_b,w_sp_b,nPt_b]=Sigma_points(pos_b,cov_b,w0);
    %[X_sp_a,w_sp_a,nPt_a]=SigmaPoints_principal_components(pos_a,cov_a,w0);
    %[X_sp_b,w_sp_b,nPt_b]=SigmaPoints_principal_components(pos_b,cov_b,w0);
    
    y_sp = zeros(3,nPt_a*nPt_b);
    w_sp = zeros(1,nPt_a*nPt_b);
    k = 1;
    for i = 1:nPt_a
        R = [cos(X_sp_a(3,i)) -sin(X_sp_a(3,i)) 0;
            sin(X_sp_a(3,i))  cos(X_sp_a(3,i)) 0;
            0              0             1];
        for j=1:nPt_b
            y_sp(:,k) = R*(X_sp_b(:,j))+X_sp_a(:,i);
            w_sp(k) = w_sp_a(i)*w_sp_b(j);
            k = k+1;
        end
    end
    pos_out = sum(y_sp.*w_sp,2);
    
    cov_out = zeros(3,3);
    for i =1:k-1
        cov_out = cov_out + w_sp(i)*(y_sp(:,i)-pos_out)*(y_sp(:,i)-pos_out)';
    end
    
end