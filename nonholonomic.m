%非ホロノミックロボットの制約を回避するために
function [omega,v] = nonholonomic(phi,rho,N)
omega=zeros(1,N);
v=zeros(1,N);
for i=1:N
    if (((-pi/2)<=phi(i) && phi(i)<=0) || (phi(i)>=0 && phi(i)<=(pi/2)))
        omega(i)=phi(i);
    else 
        omega(i)=phi(i)+pi;
    end 
end
for i=1:N
    if (((-pi/4)<=phi(i) && phi(i)<=0) || (phi(i)>=0 && phi(i)<=(pi/4)))
        v(i)=rho(i);
    elseif (pi/4)<=phi(i)&&phi(i)<=(3*pi/4)
        v(i)=rho(i)/tan(phi(i));
    elseif  (((-3*pi/4)<=phi(i)) && (phi(i)<=(-pi/4)))
        v(i)=-rho(i)/tan(phi(i));
    elseif (pi*3/4<=phi(i) && phi(i)<=pi) || (-pi<=phi(i) && phi(i)<=-pi*3/4)
        v(i)=-rho(i);
    end
end
end
