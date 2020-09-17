function zmp = calc_model_zmp(chain_model,dP,dL,zmp_z)

M = chain_model.M;
G = [0,0,-9.8];
c = chain_model.com;
zmp_x = (M*G*c(1) + zmp_z*dP(1) - dL(2)) / (M*G + dP(3));   % Kajita (3.73)
zmp_y = (M*G*c(2) + zmp_z*dP(2) + dL(1)) / (M*G + dP(3));   % Kajita (3.74)

zmp = [zmp_x,zmp_y,zmp_z];
