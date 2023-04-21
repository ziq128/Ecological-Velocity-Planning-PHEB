function p_bat = bat(delta_soc)
global Voc Qb Rbat DC_yita  
I = delta_soc * Qb * 3600;
p_bat0 = (Voc^2 - (Voc - 2*Rbat*I).^2) /(4000*Rbat);
if p_bat0 >=0
    p_bat = p_bat0 * DC_yita + I.^2 * Rbat/1000;
else
    p_bat = p_bat0 / DC_yita + I.^2 * Rbat/1000;
end
end