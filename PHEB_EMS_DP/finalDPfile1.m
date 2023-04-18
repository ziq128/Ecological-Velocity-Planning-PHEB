% DP�㷨��ϰ ����ʽPHEV�ṹ
clc;clear;
load('UDDS_drive_cycle.mat')          %load the drive cycle (P_dem,v,t)            
ua=repmat(v,8,1);
% ua=ua1; 
%%%��������
m_car=14500;g=9.8;G=m_car*g;
f=0.0076+0.000056*ua;
CdA=0.65*2.5*3.2;nT=0.97;delta=1.07;i0=13.9;
R=0.433;  %��̥�ߴ磺275/70 R22.5
N=2;  %%�ֱ�˫��� �������Ϊ2
% Voc=3.2*180;Qb=120%%%�������(A*h)
Qb = 180 ;
Rbat= 0.2;
Voc= 340; %��ز���
DC_yita=0.97;%AC-DC�任����Ч��ֵ
%%
%%�����ֱߵ��ģ�� 
load APU_Motor_data.mat
% load motor.mat
n_motor=motor(:,1);tq_motor=motor(:,2);eff_motor=motor(:,3);
F1=TriScatteredInterp(n_motor,tq_motor,eff_motor);%%�ֱߵ��Ч�ʺ���
%���뷢����ģ�͡�ISG���ģ��
% load engine.mat
n_eng=engine(:,1);tq_eng=engine(:,2);p_eng=n_eng.*tq_eng/9550;be_eng=engine(:,4);
% load isg.mat
n_isg=isg(:,1);tq_isg=isg(:,2);p_isg=n_isg.*tq_isg/9550;efficiency=isg(:,3)/100;
%%APU
%��������
n_engine=linspace(800,2800,300);
Pe=linspace(0,80,300);
[n1,Pe1]=meshgrid(n_engine,Pe);


H1=TriScatteredInterp(n_eng,p_eng,be_eng,'linear');%������ȼ�����Ĳ�ֵ����
Be=H1(n1,Pe1);%�����ڲ�ֵ����
H2=TriScatteredInterp(n_isg,p_isg,efficiency,'natural');%���Ч�ʲ�ֵ����
EFF_ISG=H2(n1,Pe1);
be_APU=Be./EFF_ISG;%APU��ȼ��������
%APU 
% figure(1);
% [c3,h3]=contour(Pe1,n1,be_APU);
% set(h3,'ShowText','On','LevelList',[218 220 223 227 230 235 240 244 259 280 300 325]);
% axis([0,80,800,2800]);
% xlabel('����/kw');ylabel('ת��/(r/min)')
% hold on;
%�ҵ��ȹ���ʱ����Сȼ����������
[m,n]=min(transpose(be_APU));
x0=Pe(1,:);
y0=zeros(1,length(n_engine));
for i=1:length(n_engine)
    y0(i)=n1(1,n(i)); %ѭ����ֵ��ȷ����Ӧ������Сȼ�������ʵ�ת��ֵ
end

% plot(x0,y0,'r','linewidth',2);
% figure(2);
% plot(x0,m);
% xlabel('����/kw');ylabel('��Сȼ��������/(g/kw*h)')

%%
P_APU=x0;Becostmin=m;
%%
%ѭ���и��ݳ��ټ���ÿ��������
kmax=length(ua);
a=diff(ua);%�������ٶ�
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
hmax=100;%SOC��ֳ�hmax�����
APU_number=80;%APU������ɢ����
socmin=0.3;%������ʣ������
socmax=0.6;%����������
P_APU_0=[0,linspace(10,80,APU_number)]; %��ɢApu�������ʵ�
DPSOC=linspace(socmin,socmax,hmax);%ÿ��ʱ����SOC���ܵ�ֵ
DPgasoline0=interp1(P_APU,Becostmin,P_APU_0);%APU������ʶ�Ӧ��ʱ�ͺ����ŵ�
DPgasoline0(isnan(DPgasoline0))=0;%����ɢ��APU��0������Ӧ���ܺ�ҲΪ0

DPcostmin1=zeros(hmax,kmax-1);
DPcostmin2=linspace(0,0.02,hmax);
DPcostmin=[DPcostmin1 DPcostmin2']; %����costmin���һ����ֵ �Ա��ڲ�ֵ

DP_APU=zeros(hmax,kmax);%APU������ʵı仯��¼
DPENGCOST=zeros(1,APU_number);%Ԥ����ռ�洢һ��ѭ����APU����������
PAPU=zeros(1,APU_number);%Ԥ����ռ�洢һ��ѭ����APU�Ĺ���
PBAT=zeros(1,APU_number);%Ԥ����ռ�洢һ��ѭ���µ�صĹ���
DPSOCNEXT=zeros(1,APU_number);%Ԥ����ռ�洢ÿʱ��ÿ��soc���¶�Ӧ��SOCֵ
DPCOST=zeros(1,APU_number); %Ԥ����ռ�洢ÿ��ʱ���µ�ÿ��·������������

Xmin=zeros(hmax,kmax);


% dbstop in finalDPfile1 at 125 if k==1026

for k=kmax-1:-1:1  
    for h=1:hmax 
        for q=1:APU_number
%           P_APU0=P_APU_0(q);
%           DPenginecost=0; 
        if  Preq(k)<0||Preq(k)==0   %���Preq<0,APU��������������������س�� %���Preq=0����ʱ�������͵�ؾ�������
            Pbat=max(Preq(k),-30);
            DPenginecost=0;
        else 
            Pbat=Preq(k)-P_APU_0(q); %Pbat����Ϊ������Ϊ�����������طŵ硢�������س��
            DPenginecost=(P_APU_0(q)*DPgasoline0(q))/3600; %APU���������� ���������ĵ�λ��Ϊg/s
        end
%         deltaSOC=(Voc-sqrt(Voc^2-4000*Rbat*Pbat))/(2*Rbat*(Qb/3600));
        %���ʵ�λ��ΪW��Qb��λ��ΪA*s;
        I_out=(Voc-sqrt(Voc^2-4000*Rbat*Pbat/DC_yita))/(2*Rbat).*(Pbat>0);%Rintģ�ͣ��ŵ������
        I_in =(Voc-sqrt(Voc^2-4000*Rbat*Pbat*DC_yita))/(2*Rbat).*(Pbat<=0);%Rintģ�ͣ���������
        Pbat_out=I_out.^2*Rbat/1000+Pbat/DC_yita.*(Pbat>0);%�ŵ�ʱ���������������ܱ�ࣻ
        Pbat_in =I_in.^2*Rbat/1000+Pbat*DC_yita.*(Pbat<=0);%���ʱ��������ɴ洢���ܱ��٣�
        Pbat=Pbat_in+Pbat_out;
        I=I_in+I_out;
        deltaSOC=I/3600/Qb;
        DPSOCnext=DPSOC(h)-deltaSOC;%�ɵ�k����SOCֵ�����k+1����SOCֵ
        
        DPSOCnext(DPSOCnext>socmax)=socmax;
        DPSOCnext(DPSOCnext<socmin)=socmin;%����߽�
        DPSOCNEXT(q)=DPSOCnext;
        
        DPENGcost=(DPenginecost/0.72/1000)*6.4;
        %DPenginecost�ĵ�λΪg/s,�����ܶ� Ϊ0.72g/ml,�ͼ�Ϊ6.4Ԫ/L DPENGcost���λΪԪ/s 
        DPcost=DPENGcost+(Pbat*0.8)/3600;%�����ܺ�
        %ȡ�ܼ�Ϊ�Ż�Ŀ�� ���Ϊ0.8Ԫ/��
        DPcost(DPSOCnext<=socmin)=0.1;%�߽�ͷ�
        DPcost(DPSOCnext>=socmax)=0.1 ;
        
        DPCOST(q)=DPcost;%����ÿһ���ĵ����ͺ�
        PBAT(q)=Pbat;
%       PAPU(q)=P_APU_0;
        DPENGCOST(q)=DPenginecost;
        
        end
        DPNEXTCOST=interp1(DPSOC,DPcostmin(:,k+1),DPSOCNEXT);
        DPcosttogo=DPCOST+DPNEXTCOST;
        
        [DPcosttogomin,X]=min(DPcosttogo);%ȷ���ܺ���͵��ֵ��λ�� DPcosttogomin��¼��ÿ��·����Сֵ
         DPcostmin(h,k)=DPcosttogomin;%��ÿ��ѭ�����ۺ��ܺ����Ŵ洢

%         X=find(DPcosttogo==DPcosttogomin);%ȷ���ܺ���͵��ֵλ��
        Xmin(h,k)=X;
        DP_APU(h,k)=P_APU_0(X);
        DPPbat(h,k)=PBAT(X);%���ŵ�����µ�صĹ������� 
        DPSOCnextmin(h,k)=DPSOCNEXT(X);%�ܺ���������¹������Ӧ��SOCֵ
        DPenginecostmin(h,k)=DPENGCOST(X);%���ŵ�����µ�λʱ�䷢����������       
    end
    disp(k);%��ʾ����
%     if k==17000
%         pause;
%     end
end
 save;