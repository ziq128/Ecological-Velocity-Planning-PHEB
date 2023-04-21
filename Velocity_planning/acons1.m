function a = acons1(vv,p_apu,delta_t,p_bat)
global m_car G CdA nT delta 
s_step = 1;
pdem = p_bat + p_apu * delta_t;
v = vv*3.6;
f = 0.0076+0.000056*v;
a =  3600 * (pdem * (vv/s_step)* nT - G*f*v/3600 - CdA*v^3/76140)/(delta*m_car*v) ;
end