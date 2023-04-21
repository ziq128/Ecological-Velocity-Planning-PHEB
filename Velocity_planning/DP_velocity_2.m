vopt_sim = zeros(1,kmax);
v_sim = zeros(1,kmax);
a_sim = zeros(1,kmax);
t_sim = zeros(1,kmax);
t_arrv_sim = zeros(1,kmax);
P_apu_sim= zeros(1,kmax); 
t2 = 1;
vopt_sim(1) = 1;
% dbstop 10 in DP_Ecorouting2_20201118 if k == 3
for kk=1:kmax-1
    a_sim(kk) = interp1(v,aoptnext(:,kk),vopt_sim(kk),'linear','extrap'); 
%     P_apu_sim(kk) = interp1(v,Papu_optnext(:,kk),vopt_sim(kk));
    vopt_sim(kk+1) = vopt_sim(kk) + a_sim(kk)* s_step/vopt_sim(kk);
    t_sim(kk+1) = 1/vopt_sim(kk+1);
    t_arrv_sim(kk+1) = t_arrv_sim(kk)+ t_sim(kk+1);
    if t2 > length(bus_stop_mark)
        continue;
    elseif kk == bus_stop_mark(t2) && t2 <=14
        t_arrv_sim(kk+1) =  t_arrv_sim(kk+1) + stop_time(t2);
        t2 = t2+1;       
    end
end

figure(5)
% plot(v_subject);
hold on;
p1 = plot(t_arrv_sim,vopt_sim);
figure(6)
load V_IDM_2.mat
p0= plot(Distance_IDM);
hold on;
p2 = plot(t_arrv_sim,Distance); 
y1=1:T*cycle;
for jj=1:n1
p3 = plot(y1,x(jj)*Tfc_sig(jj,:),'r');
hold on;
end

for qq=1:n2
p4=plot(y1,repmat(bus_stop_mark(qq),1,1500),'b:');
hold on;
end

axis([0,1500,0,6000]);
% grid on;
ylabel('Distance(m)');xlabel('Time(s)');





