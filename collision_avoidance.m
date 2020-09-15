%collision_avoidance.m
%衝突回避
function [ux2,uy2]=collision_avoidance(ux,uy,rx,ry,N)
%dp=0.2;
dp=6;
k=0.2;
%k=0.1;
K=zeros(2,N);
U=zeros(2,N);

%potential
for i=1:N
    for j=1:N
       if (i==2 && j==1) || (i==3 && j==1) || (i==3 && j==2) || (i==4 && j==1) || (i==4 && j==3) || (i==5 && j==1) || (i==5 && j==4)
           U(1,i)=U(1,i)-(2*dp*(rx(i,j))/((rx(i,j)^2+ry(i,j)^2)^1)-rx(i,j)/((rx(i,j))^2+(ry(i,j))^2));
           U(2,i)=U(2,i)-(2*dp*(ry(i,j))/((rx(i,j)^2+ry(i,j)^2)^1)-ry(i,j)/((rx(i,j))^2+(ry(i,j))^2));
           K(1,i)=K(1,i)+k*(ux(1,i)-ux(1,j));
           K(2,i)=K(2,i)+k*(uy(1,i)-uy(1,j));
      end
    end
    ux2(1,i)=ux(1,i)+U(1,i)*K(1,i);
    uy2(1,i)=uy(1,i)+U(2,i)*K(2,i);
end
end