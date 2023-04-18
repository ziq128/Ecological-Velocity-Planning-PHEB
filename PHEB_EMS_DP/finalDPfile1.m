% DP算法练习 串联式PHEV结构
clc;clear;
load('UDDS_drive_cycle.mat')          %load the drive cycle (P_dem,v,t)            
ua=repmat(v,8,1);
% ua=ua1; 
%%%整车参数
m_car=14500;g=9.8;G=m_car*g;
f=0.0076+0.000056*ua;
CdA=0.65*2.5*3.2;nT=0.97;delta=1.07;i0=13.9;
R=0.433;  %轮胎尺寸：275/70 R22.5
N=2;  %%轮边双电机 电机数量为2
% Voc=3.2*180;Qb=120%%%电池容量(A*h)
Qb = 180 ;
Rbat= 0.2;
Voc= 340; %电池参数
DC_yita=0.97;%AC-DC变换器的效率值
%%
%%输入轮边电机模型 
load APU_Motor_data.mat
% load motor.mat
n_motor=motor(:,1);tq_motor=motor(:,2);eff_motor=motor(:,3);
F1=TriScatteredInterp(n_motor,tq_motor,eff_motor);%%轮边电机效率函数
%输入发动机模型、ISG电机模型
% load engine.mat
n_eng=engine(:,1);tq_eng=engine(:,2);p_eng=n_eng.*tq_eng/9550;be_eng=engine(:,4);
% load isg.mat
n_isg=isg(:,1);tq_isg=isg(:,2);p_isg=n_isg.*tq_isg/9550;efficiency=isg(:,3)/100;
%%APU
%划分网格
n_engine=linspace(800,2800,300);
Pe=linspace(0,80,300);
[n1,Pe1]=meshgrid(n_engine,Pe);


H1=TriScatteredInterp(n_eng,p_eng,be_eng,'linear');%发动机燃油消耗插值函数
Be=H1(n1,Pe1);%网格内插值计算
H2=TriScatteredInterp(n_isg,p_isg,efficiency,'natural');%电机效率插值函数
EFF_ISG=H2(n1,Pe1);
be_APU=Be./EFF_ISG;%APU的燃油消耗率
%APU 
% figure(1);
% [c3,h3]=contour(Pe1,n1,be_APU);
% set(h3,'ShowText','On','LevelList',[218 220 223 227 230 235 240 244 259 280 300 325]);
% axis([0,80,800,2800]);
% xlabel('功率/kw');ylabel('转速/(r/min)')
% hold on;
%找到等功率时的最小燃油消耗曲线
[m,n]=min(transpose(be_APU));
x0=Pe(1,:);
y0=zeros(1,length(n_engine));
for i=1:length(n_engine)
    y0(i)=n1(1,n(i)); %循环赋值，确定对应各列最小燃油消耗率的转速值
end

% plot(x0,y0,'r','linewidth',2);
% figure(2);
% plot(x0,m);
% xlabel('功率/kw');ylabel('最小燃油消耗率/(g/kw*h)')

%%
P_APU=x0;Becostmin=m;
%%
%循环中根据车速计算每秒需求功率
kmax=length(ua);
a=diff(ua);%工况加速度
% a=repmat([a;0],8,1);
a=[a;0];
Preq0=zeros(1,kmax);
for j=1:1:kmax
      Preq0(j)=(G*f(j).*ua(j)/3600+CdA*ua(j).^3/76140+delta*m_car*ua(j).*a(j)/3600)/nT;
end
preq=Preq0';
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
%%
hmax=100;%SOC点分成hmax种情况
APU_number=80;%APU功率离散点数
socmin=0.3;%电池最低剩余容量
socmax=0.6;%电池最大容量
P_APU_0=[0,linspace(10,80,APU_number)]; %离散Apu工作功率点
DPSOC=linspace(socmin,socmax,hmax);%每个时刻下SOC可能的值
DPgasoline0=interp1(P_APU,Becostmin,P_APU_0);%APU输出功率对应此时油耗最优点
DPgasoline0(isnan(DPgasoline0))=0;%给离散的APU点0给出对应的能耗也为0

DPcostmin1=zeros(hmax,kmax-1);
DPcostmin2=linspace(0,0.02,hmax);
DPcostmin=[DPcostmin1 DPcostmin2']; %给出costmin最后一列数值 以便于插值

DP_APU=zeros(hmax,kmax);%APU输出功率的变化记录
DPENGCOST=zeros(1,APU_number);%预分配空间存储一组循环下APU的能量消耗
PAPU=zeros(1,APU_number);%预分配空间存储一组循环下APU的功率
PBAT=zeros(1,APU_number);%预分配空间存储一组循环下电池的功率
DPSOCNEXT=zeros(1,APU_number);%预分配空间存储每时刻每个soc点下对应的SOC值
DPCOST=zeros(1,APU_number); %预分配空间存储每个时刻下的每条路径的能量消耗

Xmin=zeros(hmax,kmax);


% dbstop in finalDPfile1 at 125 if k==1026

for k=kmax-1:-1:1  
    for h=1:hmax 
        for q=1:APU_number
%           P_APU0=P_APU_0(q);
%           DPenginecost=0; 
        if  Preq(k)<0||Preq(k)==0   %如果Preq<0,APU不工作，回收能量给电池充电 %如果Preq=0，此时发动机和电池均不工作
            Pbat=max(Preq(k),-30);
            DPenginecost=0;
        else 
            Pbat=Preq(k)-P_APU_0(q); %Pbat可能为正可能为负，正代表电池放电、负代表电池充电
            DPenginecost=(P_APU_0(q)*DPgasoline0(q))/3600; %APU的能量消耗 将能量消耗单位变为g/s
        end
%         deltaSOC=(Voc-sqrt(Voc^2-4000*Rbat*Pbat))/(2*Rbat*(Qb/3600));
        %功率单位变为W，Qb单位变为A*s;
        I_out=(Voc-sqrt(Voc^2-4000*Rbat*Pbat/DC_yita))/(2*Rbat).*(Pbat>0);%Rint模型，放电情况；
        I_in =(Voc-sqrt(Voc^2-4000*Rbat*Pbat*DC_yita))/(2*Rbat).*(Pbat<=0);%Rint模型，充电情况；
        Pbat_out=I_out.^2*Rbat/1000+Pbat/DC_yita.*(Pbat>0);%放电时，功损造成输出电能变多；
        Pbat_in =I_in.^2*Rbat/1000+Pbat*DC_yita.*(Pbat<=0);%充电时，功损造成存储电能变少；
        Pbat=Pbat_in+Pbat_out;
        I=I_in+I_out;
        deltaSOC=I/3600/Qb;
        DPSOCnext=DPSOC(h)-deltaSOC;%由第k步的SOC值计算出k+1步的SOC值
        
        DPSOCnext(DPSOCnext>socmax)=socmax;
        DPSOCnext(DPSOCnext<socmin)=socmin;%处理边界
        DPSOCNEXT(q)=DPSOCnext;
        
        DPENGcost=(DPenginecost/0.72/1000)*6.4;
        %DPenginecost的单位为g/s,汽油密度 为0.72g/ml,油价为6.4元/L DPENGcost最后单位为元/s 
        DPcost=DPENGcost+(Pbat*0.8)/3600;%单步能耗
        %取总价为优化目标 电费为0.8元/度
        DPcost(DPSOCnext<=socmin)=0.1;%边界惩罚
        DPcost(DPSOCnext>=socmax)=0.1 ;
        
        DPCOST(q)=DPcost;%储存每一步的单步油耗
        PBAT(q)=Pbat;
%       PAPU(q)=P_APU_0;
        DPENGCOST(q)=DPenginecost;
        
        end
        DPNEXTCOST=interp1(DPSOC,DPcostmin(:,k+1),DPSOCNEXT);
        DPcosttogo=DPCOST+DPNEXTCOST;
        
        [DPcosttogomin,X]=min(DPcosttogo);%确定能耗最低点的值的位置 DPcosttogomin记录下每条路径最小值
         DPcostmin(h,k)=DPcosttogomin;%将每个循环的综合能耗最优存储

%         X=find(DPcosttogo==DPcosttogomin);%确定能耗最低点的值位置
        Xmin(h,k)=X;
        DP_APU(h,k)=P_APU_0(X);
        DPPbat(h,k)=PBAT(X);%最优点情况下电池的功率消耗 
        DPSOCnextmin(h,k)=DPSOCNEXT(X);%能耗最优情况下工况点对应的SOC值
        DPenginecostmin(h,k)=DPENGCOST(X);%最优点情况下单位时间发动机耗油量       
    end
    disp(k);%显示进度
%     if k==17000
%         pause;
%     end
end
 save;