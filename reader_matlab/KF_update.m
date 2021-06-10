function  [x,P] = KF_update(x1,P1,y,R,C)
    K = P1*C'/(C*P1*C'+R);
    x = x1 + K*(y-C*x1);
    P = (eye(size(P1))-K*C)*P1*(eye(size(P1))-K*C)'+K*R*K';
    if x(3)>pi || x(3)<-pi
        x(3) = x(3)- sign(x(3))*2*pi;
    end
end

