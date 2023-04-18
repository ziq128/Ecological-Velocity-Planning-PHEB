load figure1.mat
load V_IDM_2.mat
load figure2.mat
x=(750:750:5250);
figure(1)
subplot(2,1,1)
S_lamda0 = cumsum(vopt_t_lamda0);
S_lamda1 = cumsum(vopt_t_lamda1);
plot(S_lamda0(find(S_lamda0 <= Distance_IDM(930))),vopt_t_lamda0(find(S_lamda0 <= Distance_IDM(930))),'--');hold on;
plot(S_lamda1(find(S_lamda1 <= Distance_IDM(930))),vopt_t_lamda1(find(S_lamda1 <= Distance_IDM(930))),'--');hold on;
plot(Distance_IDM(1:930),v_subject(1:930));

% for i = 1:1:930
%     v_subject_ub(i) = v_subject(i) + (v_subject(i)^2/1500);
%     v_subject_lb(i) = v_subject(i) - (v_subject(i)^2/1500); 
% end
% plot(cumsum(v_subject_ub), v_subject_ub,'r--');
% plot(cumsum(v_subject_lb), v_subject_lb,'r--');
legend('{\lambda} = 1','{\lambda} = 0','original')
ylabel('Velocity(m/s)');
subplot(2,1,2)
p0 = plot(Distance_IDM(1:930),(1:1:930));hold on;
for jj=1:4
p1 = plot(x(jj)*Tfc_sig(jj,:),y1,'r','LineWidth',2);
hold on;
end
for qq=1:10
p2=plot(repmat(bus_stop_mark(qq),1,1500),y1,'b:');
hold on;
end
p44 = plot(repmat(bus_stop_mark(11),1,1500),y1,'r');
Distance_lamda0(find(Distance_lamda0 <= Distance_IDM(930)))
p3 = plot(Distance_lamda0(find(Distance_lamda0 <= Distance_IDM(930))),t_arrv_sim_lamda0(find(Distance_lamda0 <= Distance_IDM(930))));hold on;
p4 = plot(Distance_lamda1(find(Distance_lamda1 <= Distance_IDM(930))),t_arrv_sim_lamda1(find(Distance_lamda1 <= Distance_IDM(930))));hold on;
legend([p1,p2,p0,p3,p4],'Red-light','Bus-stop','original','{\lambda} = 1','{\lambda }= 0','Location','best')
axis([0,4000,0,1000]);
xlabel('Distance(m)');ylabel('Time(sec)');

figure(2)
% Distance_IDM(find(Distance_IDM < 3721));
% subplot(2,1,1)
plot(cumsum(v_subject(1:930)),v_subject(1:930)*3.6);
xlabel('Distance(m)');ylabel('Velocity(km/h)');
% subplot(2,1,2)
figure(3)
for jj=1:4
p11 = plot(x(jj)*Tfc_sig(jj,:),y1,'r','LineWidth',2);
hold on;
end
for qq=1:10
p22=plot(repmat(bus_stop_mark(qq),1,1500),y1,'b:');
hold on;
end
p33 = plot(cumsum(v_subject(1:930)),(1:1:930));
p44 = plot(repmat(bus_stop_mark(11),1,1500),y1);
legend([p11,p22,p33],'Red-light','Bus-stop','original','Location','best');
axis([0,4000,0,1000]);
xlabel('Distance(m)');ylabel('Time(sec)');

figure(4)
p0 = plot(Distance_IDM(1:930));hold on;
for jj=1:4
p1 = plot(y1,x(jj)*Tfc_sig(jj,:),'r','LineWidth',2);
hold on;
end
for qq=1:10
p2=plot(y1,repmat(bus_stop_mark(qq),1,1500),'b:');
hold on;
end
p44 = plot(y1, repmat(bus_stop_mark(11),1,1500),'r');
Distance_lamda0(find(Distance_lamda0 <= Distance_IDM(930)))
p3= plot(t_arrv_sim_lamda0(find(Distance_lamda0 <= Distance_IDM(930))),Distance_lamda0(find(Distance_lamda0 <= Distance_IDM(930))));hold on;
p4 = plot(t_arrv_sim_lamda1(find(Distance_lamda1 <= Distance_IDM(930))),Distance_lamda1(find(Distance_lamda1 <= Distance_IDM(930))));hold on;
legend([p1,p2,p0,p3,p4],'Red-light','Bus-stop','original','{\lambda} = 1','{\lambda} = 0','Location','best')
axis([0,1000,0,4000]);
ylabel('Distance(m)');xlabel('Time(sec)');
