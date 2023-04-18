load('UDDS_drive_cycle.mat')          %load the drive cycle (P_dem,v,t)            
ts = 1;                               %time step
N = length(t);                        %length of time vector

preq = pheb_preq(v)*1000;

P_dem = P_dem*1000;                   %Power demand KW to W
figure(1)
plot(t,P_dem)
hold on
plot(t,preq)
title('Power demand')
xlabel('Time(sec)')
ylabel('Required Power(W)')

figure(2)
plot(t,v*3.6)
grid on;
xlabel('Time(sec)')
ylabel('Desired velocity(km/h)')
title('UDDS Speed Profile')

% % Q_batt = 
% % U_oc = 
% % Pb_max = 
% % Pe_max =
% fl_wt_en = 0.001; %no. of grams consumed per unit energy consumption
% SOC_min = 0.3;         % Lower SOC limit
% SOC_max = 0.9;         % Upper SOC limit
% SOC_grid = linspace(SOC_min,SOC_max,80)';   %SOC grid
% ns = length(SOC_grid);
% % DP
% V = zeros(ns,N);            %Value function
% V(:,N) = 0;                 %Boundary condition   
% 
% for i = N-1:-1:1
%     for j = 1:ns
%         lb = max([(((SOC_max-SOC_grid(j))*Q_batt*U_oc)/-ts),-Pb_max, Preq(i)-Pe_max]);
%         ub = min([(((SOC_min-SOC_grid(j))*Q_batt*U_oc)/-ts),Pb_max, Preq(i)]);
%         P_batt_grid = linspace(lb,ub,250);      %P_batt grid 
%         P_eng = P_dem(i) - P_batt_grid;         %P_eng at for P_batt_grid
%         c2g = (ts*fl_wt_en* P_eng)./(eng_eff(P_eng)); %costtogo
%         SOC_next = SOC_grid(j) - (ts .* P_batt_grid ./ (Q_batt*U_oc));
%         V_nxt = interp1(SOC_grid,V(:,i+1),SOC_next);
%         [V(j,i), k] = min([c2g + V_nxt]);
%         u_opt(j,i) = P_batt_grid(k); 
%         
%     end
% end
% 
% 
% [Pb_07, Pe_07, FC_07, SOC_07]= RUN_HEV(0.7,N,SOC_grid,u_opt,P_dem);
% [Pb_05, Pe_05, FC_05, SOC_05]= RUN_HEV(0.5,N,SOC_grid,u_opt,P_dem);
% [Pb_03, Pe_03, FC_03, SOC_03]= RUN_HEV(0.3,N,SOC_grid,u_opt,P_dem);
% figure;
% plot(SOC_07)
% hold on;
% plot(SOC_05)
% plot(SOC_03)
% title('SOC')
% legend('SOC 0.7','SOC 0.5','SOC 0.3')

