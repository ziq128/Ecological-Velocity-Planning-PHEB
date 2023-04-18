function [Pbat0, P_APU0,DPcost_ALL, SOCnextopt]=RUN_HEV(SOC_init,kmax, DPSOC,DPSOCnextmin, DP_APU, DPPbat, DPenginecostmin, DPcostmin)
SOCopt=zeros(1,kmax,'double');
SOCnextopt=zeros(1,kmax,'double');
P_APU0=zeros(1,kmax,'double');
Pbat0=zeros(1,kmax,'double');
DPcostmin0=zeros(1,kmax,'double');

SOCopt(1)=SOC_init;%实质为0时刻的soc值

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
        SOCnextopt(k)=SOCnextopt(k-1); %SOCnextopt表示1时刻末的SOC值，通过插值求的。
        DPcostmin0(k)=DPcostmin0(k-1);
    else
        SOCopt(k)=SOCnextopt(k-1);%将上步长的末值赋给下步长的初始值
       
        %根据此步长的初始值插值计算本步长的末值
        SOCnextopt(k)=interp1(DPSOC,DPSOCnextmin(:,k),SOCopt(k));%DPSOC表示步长初始点的SOC，DPSOCnextmin表示步长末点SOC的值
        P_APU0(k)=interp1(DPSOC,DP_APU(:,k),SOCopt(k));%发动机功率输出
        
        Pbat0(k)=interp1(DPSOC,DPPbat(:,k),SOCopt(k));%电池功率
        mdotopt(k)=interp1(DPSOC,DPenginecostmin(:,k),SOCopt(k));%单步长发动机的耗油量（g/s）
        DPcostmin0(k)=interp1(DPSOC,DPcostmin(:,k),SOCopt(k));%综合能耗累加值
    end
    DPcost_all(k)=DPcost_all(k-1)+(mdotopt(k)/0.72/1000)*6.4+(Pbat0(k)*0.8)/3600;
end
 DPcost_ALL=DPcost_all(kmax);
end 
