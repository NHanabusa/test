% formation_control_main.m
% 20190724
% 非ホロノミックロボットの動きにしてみる．
% 最終的な方位も考えた
% 衝突回避

clear;
close all;

% エージェントの数
N = 5;

% 初期位置 ワールド座標
 rng(6); x = 25*rand(1,N);
 rng(7); y = -20*rand(1,N);
 theta=[pi/10 pi/5 -pi/4 0 pi];
 %theta=zeros(1,N);
 %rng(4); theta = rand(1,N);

% 目標の相対座標　dpij: iのローカル座標系におけるjの目標位置
dp21=[-30 0];
dp31=[-15*sqrt(3) 15];
dp32=[-15*sqrt(3) -15];
dp41=[-15*sqrt(3) -15];
dp43=[0 -30];
dp51=[-30 0];
dp54=[-15 -15*sqrt(3)];

% 目標の相対距離  dd(i,j): ij間の目標距離
  dd=[0 0 0 0 0;
      norm(dp21) 0 0 0 0;
      norm(dp31) norm(dp32) 0 0 0;
      norm(dp41) 0 norm(dp43) 0 0;
      norm(dp51) 0 0 norm(dp54) 0];

sgn=1; %目標の隣接順序 符号を変えれば反転する

%% 
rx=zeros(N,N);   %相対位置　rx(i,j):  iのローカル座標系におけるjの位置(x座標)
ry=zeros(N,N);
dx=zeros(1,N);   %一時的な目標位置
dy=zeros(1,N);
K=0.3; % ゲイン
Kw=0.5;
tt=0.1; 
omega=zeros(1,N); %非ホロノミックロボットの制約を考えた角速度
omega2=zeros(1,N); %目標の相対角度に収束するための角速度
v=zeros(1,N);
X=x;
Y=y;
Theta=theta;
itheta=zeros(1,N); %偏角の積分

%% 目標の相対距離に収束するために
for t=0:tt:30
    [rx,ry]=relative_position(x,y,theta,N,rx,ry); %相対位置を求める関数
    [dx,dy]=dxdy(dx,dy,rx,ry,dd,sgn,N,itheta); %目標位置を求める関数
    ux=dx*K; %速度入力
    uy=dy*K;
   % [ux,uy]=collision_avoidance(ux,uy,rx,ry,N); %衝突回避
    [phi,rho] = cart2pol(ux,uy); %速度を極座標変換して線速度と角速度を求める
    [omega,v] = nonholonomic(phi,rho,N); %非ホロノミックロボットの制約を回避するために
   
    theta=theta+omega*tt;
    itheta=itheta+omega*tt;
    
   for i=1:N
     x(i)=x(i)+v(i)*cos(theta(i))*tt;
     y(i)=y(i)+v(i)*sin(theta(i))*tt;
   end
    X=vertcat(X,x);
    Y=vertcat(Y,y);
    Theta=vertcat(Theta,theta);
end

%% 目標の角度に収束するために
% for t=0:tt:10
%     [rx,ry]=relative_position(x,y,theta,N,rx,ry); %相対位置を求める関数
%     omega2=final_angular(omega2,rx,ry,Kw,dp21,dp31,dp32,dp41,dp43,dp51,dp54,N);
%        theta=theta+omega2*tt;
%         X=vertcat(X,x);
%         Y=vertcat(Y,y);
%         Theta=vertcat(Theta,theta); 
%      if abs(omega2(1))<10^-2 && abs(omega2(2))<10^-2 && abs(omega2(3))<10^-2 && abs(omega2(4))<10^-2 && abs(omega2(5))<10^-2 %全ての角速度が小さくなったら
%       break;
%      end
% end

%% アニメーションとグラフ
%アニメーション

ii = 1; jj = 1; k=0;
while ii <length(X(:,1))
 plot(X(ii,:),Y(ii,:),'ko','MarkerFaceColor','g','MarkerSize',8,'LineWidth',2)
 hold on
 for k=1:N
 p1x(k)=X(ii,k)+cos(Theta(ii,k))*3;
 p1y(k)=Y(ii,k)+sin(Theta(ii,k))*3;
 p2x(k)=X(ii,k)+cos(Theta(ii,k))*(-1.5)-sin(Theta(ii,k))*1.5;
 p2y(k)=Y(ii,k)+sin(Theta(ii,k))*(-1.5)+cos(Theta(ii,k))*1.5;
 p3x(k)=X(ii,k)+cos(Theta(ii,k))*(-1.5)-sin(Theta(ii,k))*(-1.5);
 p3y(k)=Y(ii,k)+sin(Theta(ii,k))*(-1.5)+cos(Theta(ii,k))*(-1.5);
 end
 for k=1:N
 plot([p1x(k),p2x(k)],[p1y(k),p2y(k)],'b')
 plot([p1x(k),p3x(k)],[p1y(k),p3y(k)],'b')
 plot([p2x(k),p3x(k)],[p2y(k),p3y(k)],'b')
 end
 hold off
 
 grid on
 xlabel('x [cm]','Fontsize',20);
 ylabel('y [cm]','Fontsize',20);
 axis([-20 60 -40 40])
 set(gcf, 'Position', [200 100 650 600]);
 if ii == 1
  while jj < 30
   drawnow;
   plot_movie(jj) = getframe(gcf);
   jj = jj+1;
  end
  pause(2);
  ii = ii+1;
 else
  ii=ii+1;
 end
 drawnow;
 plot_movie(jj) = getframe(gcf);
 jj = jj+1;
end
%グラフ
 for k=1:N
 f1x(k)=X(1,k)+cos(Theta(1,k))*3;
 f1y(k)=Y(1,k)+sin(Theta(1,k))*3;
 f2x(k)=X(1,k)+cos(Theta(1,k))*(-1.5)-sin(Theta(1,k))*1.5;
 f2y(k)=Y(1,k)+sin(Theta(1,k))*(-1.5)+cos(Theta(1,k))*1.5;
 f3x(k)=X(1,k)+cos(Theta(1,k))*(-1.5)-sin(Theta(1,k))*(-1.5);
 f3y(k)=Y(1,k)+sin(Theta(1,k))*(-1.5)+cos(Theta(1,k))*(-1.5);
 end

plot(X(:,1:N),Y(:,1:N),'LineWidth',1.5)
hold on
for k=1:N
 plot([f1x(k),f2x(k)],[f1y(k),f2y(k)],'k','LineWidth',1)
 plot([f1x(k),f3x(k)],[f1y(k),f3y(k)],'k','LineWidth',1)
 plot([f2x(k),f3x(k)],[f2y(k),f3y(k)],'k','LineWidth',1)
 plot([p1x(k),p2x(k)],[p1y(k),p2y(k)],'b','LineWidth',1.5)
 plot([p1x(k),p3x(k)],[p1y(k),p3y(k)],'b','LineWidth',1.5)
 plot([p2x(k),p3x(k)],[p2y(k),p3y(k)],'b','LineWidth',1.5)
 end
 hold off
axis([-20 60 -30 40])
set(gcf, 'Position', [200 100 650 600]);
 grid 
 legend('1','2','3','4','5');
 set(gca,'FontSize',20);
xlabel('x [cm]','Fontsize',20)
ylabel('y [cm]','Fontsize',20)
