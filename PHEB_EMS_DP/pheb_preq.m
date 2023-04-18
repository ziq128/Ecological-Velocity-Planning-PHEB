function preq = pheb_preq(cyc_v)
% Computes the drive cycle demand
%     cyc_v  = velocity vector (km/h)
%     preq  = demande power vector (W)
% required power
ua = cyc_v;  % km/h
kmax=length(ua);
a=diff(ua/3.6);% acceleration (m/s2)
a=[a;0];

% PHEB Vehicle Parameters
m_car=14500;
g=9.8;
G=m_car*g;
f=0.0076+0.000056*ua;
CdA=0.65*2.5*3.2;
nT=0.97;
delta=1.07;
i0=13.9;
R=0.433;  %轮胎尺寸：275/70 R22.5
N=2;          %轮边双电机 电机数量为2
Qb = 180;     %Battery capacity (Ah)
Rbat= 0.2;
Voc =340;     %Battery Voltage(V)
DC_yita=0.97; %AC-DC变换器的效率值

Preq0=zeros(1,kmax);
for j=1:1:kmax
      Preq0(j)=(G*f(j).*ua(j)/3600+CdA*ua(j).^3/76140+delta*m_car*ua(j).*a(j)/3600)/nT;
end
preq=Preq0';

load APU_Motor_data.mat
% load motor.mat
n_motor=motor(:,1);tq_motor=motor(:,2);eff_motor=motor(:,3);
F1=TriScatteredInterp(n_motor,tq_motor,eff_motor);%%轮边电机效率函数


N_motor=ua*i0/0.377/R;
Ttq_motor=preq*9550./N_motor;
Ttq_motor(preq==0)=0;
% figure(3)
% plot(N_motor,Ttq_motor);

Preq1=preq./F1(N_motor,Ttq_motor/N).*(Ttq_motor>0);
Preq2=preq.*F1(N_motor,-Ttq_motor/N).*(Ttq_motor<0);
Preq1(isnan(Preq1))=preq(isnan(Preq1)).*(Ttq_motor(isnan(Preq1))>0);%
Preq2(isnan(Preq2))=preq(isnan(Preq2)).*(Ttq_motor(isnan(Preq2))<0);%
Preq=Preq1+Preq2;

end

