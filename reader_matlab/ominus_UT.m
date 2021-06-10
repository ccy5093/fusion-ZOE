function [pos_out, cov_out] = ominus_UT(pos_in, cov_in,w0)
    if nargin<3
        w0 = 0.2;
    end
    [X_sp,w_sp,nPt]=Sigma_points(pos_in,cov_in,w0);
    %[X_sp,w_sp,nPt]=SigmaPoints_principal_components(pos_in,cov_in,w0);

    y_sp = zeros(3,nPt);
    for i = 1:nPt
        R = [cos(X_sp(3,i)) sin(X_sp(3,i)) 0;
            -sin(X_sp(3,i))  cos(X_sp(3,i)) 0;
            0              0             1];
        y_sp(:,i) = R*(-X_sp(:,i));
    end
    pos_out = sum(y_sp.*w_sp,2);

    cov_out = zeros(3,3);
    for i =1:nPt
        cov_out = cov_out + w_sp(i)*(y_sp(:,i)-pos_out)*(y_sp(:,i)-pos_out)';
    end
end