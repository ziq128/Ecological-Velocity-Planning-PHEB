function [Pbat0, P_APU0,DPcost_ALL, SOCnextopt]=RUN_HEV(SOC_init,kmax, DPSOC,DPSOCnextmin, DP_APU, DPPbat, DPenginecostmin, DPcostmin)
SOCopt=zeros(1,kmax,'double');
SOCnextopt=zeros(1,kmax,'double');
P_APU0=zeros(1,kmax,'double');
Pbat0=zeros(1,kmax,'double');
DPcostmin0=zeros(1,kmax,'double');

SOCopt(1)=SOC_init;%ʵ��Ϊ0ʱ�̵�socֵ

SOCnextopt(1)=interp1(DPSOC,DPSOCnextmin(:,1),SOCopt(1));
P_APU0(1)=interp1(DPSOC,DP_APU(:,1),SOCopt(1));
Pbat0(1)=interp1(DPSOC,DPPbat(:,1),SOCopt(1));
mdotopt(1)=interp1(DPSOC,DPenginecostmin(:,1),SOCopt(1));
DPcostmin0(1)=interp1(DPSOC,DPcostmin(:,1),SOCopt(1));
DPcost_all=zeros(1,kmax,'double');

for k=2:kmax
    if k==kmax
        SOCopt(k)=SOCnextopt(k-1);
        mdotopt(k)=0;
        SOCnextopt(k)=SOCnextopt(k-1); %SOCnextopt��ʾ1ʱ��ĩ��SOCֵ��ͨ����ֵ��ġ�
        DPcostmin0(k)=DPcostmin0(k-1);
    else
        SOCopt(k)=SOCnextopt(k-1);%���ϲ�����ĩֵ�����²����ĳ�ʼֵ
       
        %���ݴ˲����ĳ�ʼֵ��ֵ���㱾������ĩֵ
        SOCnextopt(k)=interp1(DPSOC,DPSOCnextmin(:,k),SOCopt(k));%DPSOC��ʾ������ʼ���SOC��DPSOCnextmin��ʾ����ĩ��SOC��ֵ
        P_APU0(k)=interp1(DPSOC,DP_APU(:,k),SOCopt(k));%�������������
        
        Pbat0(k)=interp1(DPSOC,DPPbat(:,k),SOCopt(k));%��ع���
        mdotopt(k)=interp1(DPSOC,DPenginecostmin(:,k),SOCopt(k));%�������������ĺ�������g/s��
        DPcostmin0(k)=interp1(DPSOC,DPcostmin(:,k),SOCopt(k));%�ۺ��ܺ��ۼ�ֵ
    end
    DPcost_all(k)=DPcost_all(k-1)+(mdotopt(k)/0.72/1000)*6.4+(Pbat0(k)*0.8)/3600;
end
 DPcost_ALL=DPcost_all(kmax);
end 
