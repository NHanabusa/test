%% 目標位置を求める関数
%rx(i,j)がiから見たjの相対位置　ddが目標の相対距離　dx(i),dy(i)がエージェントiのローカル座標系における目標位置


function [dx,dy]=dxdy2(dx,dy,rx,ry,dd,sgn,N)

 dx(2)=rx(2,1)+dd(2,1);
 dy(2)=ry(2,1);

 for i=2:N-1 %i+1が子．その隣接する点は１とi
        % 隣接点が同じ位置
        if sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)==0    
            dx(i+1)=rx(i+1,i)+dd(i+1,i);
            dy(i+1);
        end
        
        % 隣接点が近すぎる
        if 0<sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)&&sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)<abs(dd(i+1,1)-dd(i+1,i))
            dx(i+1)=rx(i+1,i)-dd(i+1,i)*(rx(i+1,1)-rx(i+1,i))/sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2);
            dy(i+1)=ry(i+1,i)-dd(i+1,i)*(ry(i+1,1)-ry(i+1,i))/sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2);
        end
        
        % 好位置
         if abs(dd(i+1,1)-dd(i+1,i))<=sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)&&sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)<=abs(dd(i+1,1)+dd(i+1,i))
             costheta=(dd(i+1,i)^2+((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)-dd(i+1,1)^2)/(2*dd(i+1,i)*sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
             sintheta=sqrt(1-costheta^2);
             R=[costheta sgn*sintheta; -sgn*sintheta costheta]; %回転行列
             dx(i+1)=rx(i+1,i)+(R(1,1)*(rx(i+1,1)-rx(i+1,i))+R(1,2)*(ry(i+1,1)-ry(i+1,i)))*dd(i+1,i)/(sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
             dy(i+1)=ry(i+1,i)+(R(2,1)*(rx(i+1,1)-rx(i+1,i))+R(2,2)*(ry(i+1,1)-ry(i+1,i)))*dd(i+1,i)/(sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
         end
         
        % 隣接点が遠すぎる
         if abs(dd(i+1,1)+dd(i+1,i))<sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)
             dx(i+1)=rx(i+1,i)+dd(i+1,i)*(rx(i+1,1)-rx(i+1,i))/(sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
             dy(i+1)=ry(i+1,i)+dd(i+1,i)*(ry(i+1,1)-ry(i+1,i))/(sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
         end
 end
end