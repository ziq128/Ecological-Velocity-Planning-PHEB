load matlab.mat

t = [1:1:kmax];

[Pbat03, P_APU03, DPcost_all03, SOCnextopt03] = RUN_HEV(0.3, kmax, DPSOC,DPSOCnextmin, DP_APU, DPPbat, DPenginecostmin, DPcostmin);

[Pbat04, P_APU04, DPcost_all04, SOCnextopt04] = RUN_HEV(0.4, kmax,DPSOC,DPSOCnextmin, DP_APU, DPPbat, DPenginecostmin, DPcostmin);

[Pbat05, P_APU05, DPcost_all05, SOCnextopt05] = RUN_HEV(0.5, kmax,DPSOC,DPSOCnextmin, DP_APU, DPPbat, DPenginecostmin, DPcostmin);

[Pbat06, P_APU06, DPcost_all06, SOCnextopt06] = RUN_HEV(0.6, kmax,DPSOC,DPSOCnextmin, DP_APU, DPPbat, DPenginecostmin, DPcostmin);
figure(1)
plot(t, SOCnextopt03);
hold on;
plot(t, SOCnextopt04);
hold on;
plot(t, SOCnextopt05);
hold on;
plot(t, SOCnextopt06);
xlabel('Time(secs)')
ylabel('SOC')
legend('SOC_{0}=0.3','SOC_{0}=0.4','SOC_{0}=0.5','SOC_{0}=0.6')

figure(2)
subplot(5, 1, 1)
plot(t, ua);
ylabel('Speed(m/s)')
subplot(5, 1, 2)
plot(t, a)
ylabel('Acceleration(m/s^2)')
subplot(5, 1, 3)
plot(t, Preq);
ylabel('Required Power(kW)');
subplot(5, 1, 4)
plot(t, P_APU04);
hold on;
plot(t, P_APU06);
ylabel('APU Power(kW)');
subplot(5, 1, 5)
plot(t, Pbat04);
hold on;
plot(t, Pbat06);
xlabel('Time(secs)');
ylabel('Battery Power(kW)');

figure(3)
bar([DPcost_all03, DPcost_all04, DPcost_all05, DPcost_all06]);
ylabel('cost($)')





