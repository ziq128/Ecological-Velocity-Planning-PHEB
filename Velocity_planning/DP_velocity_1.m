clear;
load v_soc_boundray.mat
% load v_ccbc_bound.mat
setGlobal();
global P_APU_optimal Becost_optimal
x=(750:750:5250);
n1=size(x,2);
T_r=20;T=60; %signal cycle timing 60s
T_g=40;% Green timing
offset=[15,25,0,5,10,20,10];
Tfc_sig_0=[ones(1,T_r),NaN(1,T_g)];
cycle=25;
Tfc_sig_i =repmat(Tfc_sig_0,[1,cycle]);
for ii= 1:1:n1
    Tfc_sig(ii,:)=[NaN(1,offset(ii)),Tfc_sig_i(1:T*cycle-offset(ii))];
    if offset(ii) == 0 
        Tfc_sig(ii,:) = [Tfc_sig_i];
    else
        Tfc_sig(ii,:)= [Tfc_sig_0(1,offset(ii)+1:end),Tfc_sig_i(1:T*(cycle-1)+offset(ii))];
    end    
end 
load bus_stop.mat
bus_stop_mark = [1,ceil(mark_bus_stop(1:end-1))];
stop_time =[30,stop_time(1:end-1)];
n2 = length(bus_stop_mark);
%% Divide the state space
Distance = (1:1:5893); s_step=1;
vmin=0.001; vmax= 60/3.6; %%state speed 
% v_stop = 0.001;
kmax = size(Distance,2) ;
imax  = 40; jmax = 40;
amax = 5; amin = - 5;
Papu_max = 80; Papu_min = 0;
v = linspace(vmin,vmax,imax);%%  state space of v
J_min = [zeros(imax,kmax-1),linspace(0.1,0.2,imax)'];
tf = 1337;

t_arrv = zeros(imax,kmax);  %t_arrv represent the arrival time of every stage
aoptnext = zeros(imax, kmax);
voptnext = zeros(imax, kmax);

sk_v = zeros(jmax,imax);
delta_t = zeros(jmax,imax);
t_next= zeros(jmax,imax);  % represent the time of the next step cost
X = zeros(jmax,imax);
% lamda = 0.5;  
lamda = 0; 
t1 = n1 ; %represent the intersection index
t2 = n2; %represent the busstop index
zone = 10;
for k = kmax-1:-1:1 
    for i = 1:imax
        v_ub =  v_distance_ub(k+1);
        v_lb =  v_distance_lb(k+1);
          if k == bus_stop_mark(t2)
                v_ub = 0.001;
                v_lb = 0.001;
                t_arrv(i,k+1) = t_arrv(i,k+1) + stop_time(t2);  
                t2 = t2 - 1;
          end
          if t2 == 0
              t2 = 1;
          end
          delta_soc = SOC_distance(k+1)- SOC_distance(k);     % delta_soc/m
          P_bat = bat(delta_soc);    %P_bat/m
          tmax0 = (tf - t_arrv(i,k+1))/(k-0.05);         
          if  k >= x(t1) - zone &&  k < x(t1)  && t1 >= 1 % entre the intersection
             cp = mod(t_arrv(i,k+1)+ offset(t1),T);
              if cp - T_r > 0
                  delta_tmin  =  min([(cp - T_r)/(x(t1) - k),(T - cp)/(x(t1) - k)]);
                  delta_tmax  =  max([(cp - T_r)/(x(t1) - k),(T - cp)/(x(t1) - k)]);
              else
                   delta_tmin = nan;
                   delta_tmax = nan;
              end
%            ub = min([(v_ub- v(i))* v(i)/s_step, amax, acons1(v(i),Papu_max,delta_tmax,P_bat)]) ;
%            lb = max([(v_lb- v(i))* v(i)/s_step, amin, acons1(v(i),Papu_max,delta_tmin,P_bat)]) ; 
          ub = min([(v_ub- v(i))* v(i)/s_step, amax,(2*(s_step - v(i)*delta_tmin ))/delta_tmin ^2,(2*(s_step + v(i)*delta_tmin ))/delta_tmin ^2]) ;
          lb = max([(v_lb- v(i))* v(i)/s_step, amin,(2*(s_step - v(i)*delta_tmax ))/delta_tmax ^2,(2*(s_step + v(i)*delta_tmin ))/delta_tmin ^2]) ;
          else
%             ub = min([(v_ub - v(i))* v(i)/s_step, amax, acons1(v(i),Papu_max,tmax0,P_bat)]) ;
%             lb = max([(v_lb- v(i))* v(i)/s_step, amin, acons1(v(i),Papu_min,tmax0,P_bat)]) ; 
          ub = min([(v_ub- v(i))* v(i)/s_step, amax]) ;
          lb = max([(v_lb- v(i))* v(i)/s_step, amin,(2*(s_step - v(i)*tmax0))/tmax0]) ;
          end
          a_grid(:,i) = linspace(lb,ub,jmax);
             for j = 1:1:jmax              
              sk_v(j,i) = v(i) + a_grid(j)* s_step/v(i);
              delta_t(j,i) = s_step/ sk_v(j,i);
              t_next(j,i) = t_arrv(i,k+1) + delta_t(j,i);
              Pdem(j,i) = preq(v(i),a_grid(j));

             X(j,i) =  delta_t(j,i) * Pdem(j,i);
             end
%             g_k(:,i) = lamda * normalize(X(:,i),'range') + (1-lamda) * normalize(t_next(:,i),'range');
             g_k(:,i) = lamda * zscore(X(:,i)) + (1-lamda) * zscore(t_next(:,i));
              V_nxt(:,i) = interp1(v,sk_v(:,i),J_min(:,k+1));  %  V represent the cost of next step
              J(:,i) = g_k(:,i) + V_nxt(:,i);
              %% optimal choice
              [J_min(i,k),subopt] = min (J(:,i));              
%              Papu_optnext(i,k) = Papu(subopt,i);
              aoptnext(i,k) = a_grid(subopt,i);
              voptnext(i,k) = sk_v(subopt,i);
              t_arrv(i,k) =  t_next(subopt,i);
    end
    disp(k);
    if k == x(t1)- zone
        t1 = t1-1;
    end
    if t1 == 0
        t1 = 1;
    end
end

        



