%% Build Actual Road
%% Based on the CCBC cycle, set intersections and bus stops
%% The CCBC cycle includes the part where the speed is 0, which can be considered as the bus stop
%% Set intersection information based on driving distance
%% Simulate SPaT information for 7 intersections
clc;clear;

x=(750:750:5250); %  7 intersections
n=size(x,2);%set intersection 

T=60; %signal cycle timing60s
T_g=40;% Green timing
% T_y=3;% yellow timing
% T_r=T-T_g-T_y;% red timing
T_r = 20;

offset=[15,25,0,5,10,25,10]; %Phase difference calculation

Tfc_sig_0=[ones(1,T_r),NaN(1,T_g)]; % the first cycle for traffic light
cycle=25;
Tfc_sig_i = repmat(Tfc_sig_0,[1,cycle]); % Initial cycle for traffic light

for i= 1:1:n
    Tfc_sig(i,:)=[NaN(1,offset(i)),Tfc_sig_i(1:T*cycle-offset(i))];
    if offset(i) == 0 
        Tfc_sig(i,:) = [Tfc_sig_i];
    else
        Tfc_sig(i,:)=[Tfc_sig_0(1,offset(i)+1:end),Tfc_sig_i(1:T*(cycle-1)+offset(i))];
    end
     
end  
y1=1:T*cycle;
%% plot SPaT information
figure(1)
for j=1:n
 plot(y1,x(j)*Tfc_sig(j,:),'DisplayName','');
hold on;
% plot(x(3)*Tfc_sig(2,:),y1);
% hold on;
end
axis([0,1500,0,6500]);
% grid on;
ylabel('Distance(m)');xlabel('Time(s)');
hold on;
%% CCBC
load('GK_ChinaCCBC.mat');
% L = 200; %%unit/m/ Distance from loop detector to stop line
V = GK_China/3.6; %% km/h convert to m/s
S = cumsum(V);
%% Find the position of busstop 
for q=2:length(V)-1
if V(q)==0 && V(q-1)~=0 && V(q+1)==0
    mark_1(q) = q; 
end
if V(q)==0 && V(q-1)==0 && V(q+1)~=0
    mark_2(q) = q;
end
end
Position_1=find(mark_1~=0);
Position_2=find(mark_2~=0);Position_2 = [Position_2(2:end),1314];
stop_time = Position_2 - Position_1;
for qq=1:length(Position_1)
    mark_bus_stop(qq)=sum(V(1:Position_1(qq)));
    p2=plot(y1,repmat(mark_bus_stop(qq),1,1500),'LineStyle',':');
    hold on;
end
Distance_front=zeros(2,length(V));  %第一行代表距离 %第二行代表状态（Stss）
%%if Stss(D)==0 no stopline or traffic signal is green
%%if Stss(D)==1 stopline or traffic signal is red
Distance_front(1)=0;
for jj=1:length(V)-1
    Distance_front(1,jj+1) = Distance_front(1,jj)+V(jj);
    Distance_front(2,jj+1) = 0; 
    while find(Distance_front(1,jj+1) == mark_bus_stop)     
    Distance_front(2,jj+1) = 1;
    break;
    end      
end
time=(1:length(V));
plot(time,Distance_front(1,:));hold on;
% position_inter_stop=sort([mark_bus_stop,x(2:end-1)]);
%% Set the preceding car's velocity profile
for t=1:length(x)          %%Find the moment that arrive at the intersection 
[~,n]=find(Distance_front>=x(t));
A(t)=n(1);
end
B=A-1;B(2,:)=A; B(3,:)=x(1:end);
B=B(:,1:end);% 代表着距离与交叉口相交的最近时刻(s)
for tt=1:length(B)
tn_L(tt)= Tfc_sig(tt,B(1,tt)); %Find the phase state
if tn_L(tt)==1
    Tn_L = Tfc_sig(tt,B(1,tt):B(1,tt)+T_r);
    [~,tn_T_g(tt)] = size(find(Tn_L == 1));   %Find the time when thelight turns green
else
    tn_T_g(tt) = 0;        
end
end
B(4,:)=tn_L;
B(5,:)=tn_T_g;
B(:,8)=[0,0,6000,1,inf];

%% Modified Intelligent Driver Model
a_max = 2.5;a_c = 2.8; %%maximum acceleration=2.5 deceleration=-2.8
T_hw =1.5; %%safe headway 
V_max = max(V);
D_des_min=7.5;%% Jam distance  (the minimal vehicle distance)/m
D_sf(1) = 10 +(15-10)*rand(1,1);
v_subject(1)=0;
D_v = 50;
times_inter=1; calcu_red_t=0;
times_stop=1;
% ii=1;
%times_2=1;
Distance_subject(1)=0;
vv=1; 
%  dbstop in Crossing at 142 if vv==1328
while Distance_subject(vv) < S(end) %variable is speed 
    Distance_pre(vv) = Distance_subject(vv) + D_v;
     while Distance_pre(vv) > B(3,times_inter)-15  && Distance_pre(vv)<= B(3,times_inter) %%遇到交叉口
      if B(4,times_inter)==1
         ii_i1=vv;  
         T_rest_r=B(5,times_inter);   
        for ii_i1 = ii_i1:ii_i1+ T_rest_r
            delta_v(ii_i1)= v_subject(ii_i1)-V(ii_i1);
            D_middle(ii_i1)= v_subject(ii_i1)*T_hw + (v_subject(ii_i1)*delta_v(ii_i1))/(2*sqrt(a_max*a_c));
            D_des(ii_i1) = D_des_min + max(D_middle(ii_i1),0);
            %                 D_des(ii_i1) = D_des_min + v_subject(ii_i1)*T_hw - v_subject(ii_i1)*D_sf(ii_i1)/(2*sqrt(a_max*a_c));
            %                a(ii_i1)= a_max*[1-(v_subject(ii_i1)/V_max)^4-(D_des(ii_i1)/(L-D_sf(ii_i1)))^2];
            a(ii_i1)= -V(ii_i1)^2/(2*D_sf(ii_i1));
            v_subject(ii_i1+1)=v_subject(ii_i1)+a(ii_i1);
            if v_subject(ii_i1+1)<0.5
                v_subject(ii_i1+1)=0;
            end
            D_sf(ii_i1+1)=D_sf(ii_i1)+(V(ii_i1+1)-v_subject(ii_i1+1));
            Distance_subject(ii_i1+1)=Distance_subject(ii_i1)+v_subject(ii_i1+1);
        end
        calcu_red_t = calcu_red_t+T_rest_r;
        vv=ii_i1+1;
        Distance_pre(vv) = Distance_subject(vv) + D_v;
        times_inter=times_inter+1;
        continue; 
      else
          delta_v(vv)= v_subject(vv)-V(vv);
          D_middle(vv)= v_subject(vv)*T_hw + (v_subject(vv)*delta_v(vv))/(2*sqrt(a_max*a_c));
          D_des(vv) = D_des_min + max(D_middle(vv),0);
          a(vv)= a_max*[1-(v_subject(vv)/V_max)^4-(D_des(vv)/D_sf(vv))^2];
          v_subject(vv+1)=v_subject(vv)+a(vv);
          if v_subject(vv+1)<0.5
              v_subject(vv+1)=0;
          end
          D_sf(vv+1)=D_sf(vv)+(V(vv+1)-v_subject(vv+1));
          Distance_subject(vv+1)=Distance_subject(vv)+v_subject(vv+1);
          vv=vv+1;
          Distance_pre(vv) = Distance_subject(vv) + D_v;
          times_inter=times_inter+1;
          continue; 
      end        
     end
     if vv>=length(V)
         if Distance_subject(vv) < Distance_front(1,1304)- D_v
%          while Distance_pre(vv)>= mark_bus_stop(end) && Distance_pre(vv)< S%遇到公交站台       
             delta_v(vv)= v_subject(vv)-V(vv-calcu_red_t);
             D_middle(vv)= v_subject(vv)*T_hw + (v_subject(vv)*delta_v(vv))/(2*sqrt(a_max*a_c));
             D_des(vv) = D_des_min + max(D_middle(vv),0);
             a(vv)= a_max*[1-(v_subject(vv)/V_max)^4-(D_des(vv)/D_sf(vv))^2]
             a(vv)= -V(vv-calcu_red_t)^2/(2*D_sf(vv));
             v_subject(vv+1)=v_subject(vv)+a(vv);
             if v_subject(vv+1)<0.5
                 v_subject(vv+1)=0;
             end
             D_sf(vv+1)=D_sf(vv)+(V(vv-calcu_red_t+1)-v_subject(vv+1));
             Distance_subject(vv+1)=Distance_subject(vv)+v_subject(vv+1);
             vv=vv+1;
             Distance_pre(vv) = Distance_subject(vv) + D_v;
            continue;
         else
             delta_v(vv)= v_subject(vv)-V(vv-27);   %%27s 代表全程因为交叉口红灯的延迟时间 相当于前车的CCBC轨迹并未考虑交叉口的情况
             D_middle(vv)= v_subject(vv)*T_hw + (v_subject(vv)*delta_v(vv))/(2*sqrt(a_max*a_c));
             D_des(vv) = D_des_min + max(D_middle(vv),0);
             a(vv)= -V(vv-27)^2/(2*D_sf(vv))
             v_subject(vv+1)=v_subject(vv)+a(vv);
             if v_subject(vv+1)<0.5
                 v_subject(vv+1)=0;
             end
%              D_sf(vv+1)=D_sf(vv)+(V(vv-26)-v_subject(vv+1));
            
             Distance_subject(vv+1)=Distance_subject(vv)+v_subject(vv+1);
             D_sf(vv+1)= S(end) - Distance_subject(vv+1);
             vv=vv+1;
             Distance_pre(vv) = Distance_subject(vv) + D_v; 
             if vv+1==1338
                 break;
             else           
                 continue;
             end
         end
             
     end 
    delta_v(vv)= v_subject(vv)-V(vv);
    D_middle(vv)= v_subject(vv)*T_hw + (v_subject(vv)*delta_v(vv))/(2*sqrt(a_max*a_c));
    D_des(vv) = D_des_min + max(D_middle(vv),0);
    a(vv)= a_max*[1-(v_subject(vv)/V_max)^4-(D_des(vv)/D_sf(vv))^2]; 
    v_subject(vv+1)=v_subject(vv)+a(vv);
    if v_subject(vv+1)<0.5
        v_subject(vv+1)=0;
    end
    D_sf(vv+1)=D_sf(vv)+(V(vv+1)-v_subject(vv+1));  
    Distance_subject(vv+1)=Distance_subject(vv)+v_subject(vv+1);      
    vv=vv+1;  
    end
      
v_subject(find(v_subject<0))=0;
Distance_IDM = cumsum(v_subject);
time=(1:length(v_subject));
y2=plot(time,Distance_IDM);hold on;

figure(2)
plot(S/1000,GK_China);
hold on;
plot(Distance_IDM/1000,v_subject*3.6);
xlabel('Distance(km)');ylabel('velocity/(km/h)');

save V_IDM_2.mat v_subject Distance_IDM




