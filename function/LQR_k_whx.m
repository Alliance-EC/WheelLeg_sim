function K = LQR_k_whx(L0)
%LQR_k_whx
%    K = LQR_k_whx(L0)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2024-04-20 00:21:04

t2 = L0.^2;
t3 = L0.^3;
t4 = t2.^2;
mt1 = [L0.*(-1.922771159132324e+2)+t2.*1.691779433001481e+2-t3.*1.069256344688194e+2+t4.*7.747638754783392-6.006731054879026];
mt2 = [L0.*2.098272300180793e+2-t2.*8.586243639439053e+2+t3.*1.563572704476008e+3-t4.*1.089119015780934e+3+1.072940554554044e+1];
mt3 = [L0.*(-1.456376854811606e+1)-t2.*6.528011804371633e+1+t3.*1.055547391618225e+2-t4.*7.341192054246483e+1-9.693889932718042e-1];
mt4 = [L0.*2.628073140084111e+1-t2.*6.769688038572934e+1+t3.*9.601919710735562e+1-t4.*5.588436356630777e+1+1.588607993236016];
mt5 = [L0.*(-4.431754757132075e+1)+t2.*1.057327469938534e+2-t3.*1.138308642332896e+2+t4.*4.109502121560282e+1-1.358777664948975e+1];
mt6 = [L0.*(-7.94330594994253e+1)+t2.*1.900214611571776e+1+t3.*2.277390970941962e+2-t4.*2.688404024130205e+2+3.263795846442449e+1];
mt7 = [L0.*(-2.615523552626111e+1)-t2.*8.241702888586305e-1+t3.*8.109536659211132e+1-t4.*9.350414065149094e+1-1.171845948344264e+1];
mt8 = [L0.*(-6.24562263920951e+1)+t2.*6.053699428774009e+1+t3.*4.019881919915844e+1-t4.*8.677564509339794e+1+2.529332942881674e+1];
mt9 = [L0.*(-1.78099204744331e+2)+t2.*2.018111811961528e+2+t3.*5.565532421231877e+1-t4.*2.090896050044489e+2+6.512663695772493e+1];
mt10 = [L0.*4.005893836647786e+2-t2.*1.131162236231524e+3+t3.*1.581135262318974e+3-t4.*8.800877084612345e+2+7.747977434737851e+1];
mt11 = [L0.*(-1.765418636325565e+1)+t2.*3.977023254770091e+1-t3.*5.151190363785788e+1+t4.*2.915252041709117e+1+6.279807683583122];
mt12 = [L0.*3.501260802060359e+1-t2.*9.218640917732792e+1+t3.*1.234114579926268e+2-t4.*6.65774535078946e+1+1.272060596007228];
K = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12],2,6);
end
