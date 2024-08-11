function spd = leg_spd(dphi1,dphi4,phi1,phi4)
%LEG_SPD
%    SPD = LEG_SPD(DPHI1,DPHI4,PHI1,PHI4)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2024-04-14 09:10:26

t2 = cos(phi1);
t3 = cos(phi4);
t4 = sin(phi1);
t5 = sin(phi4);
t6 = imag(t2);
t7 = real(t2);
t8 = imag(t4);
t9 = real(t4);
t10 = t2.*(3.0./2.0e+1);
t11 = t3.*(3.0./2.0e+1);
t12 = t4.*(3.0./2.0e+1);
t13 = t5.*(3.0./2.0e+1);
t21 = t4.*1.5e-1i;
t23 = t2.*(8.1e+1./1.0e+3);
t24 = t3.*(8.1e+1./1.0e+3);
t25 = t4.*(8.1e+1./1.0e+3);
t26 = t5.*(8.1e+1./1.0e+3);
t14 = -t10;
t15 = -t11;
t16 = -t13;
t17 = t6.*(3.0./2.0e+1);
t18 = t7.*(3.0./2.0e+1);
t19 = t8.*(3.0./2.0e+1);
t20 = t9.*(3.0./2.0e+1);
t27 = -t23;
t28 = -t25;
t22 = -t18;
t29 = t12+t16;
t31 = t11+t14+3.0./2.0e+1;
t30 = t29.^2;
t32 = t31.^2;
t33 = t30.*2.916e-1;
t34 = t32.*2.916e-1;
t35 = t30+t32;
t36 = t35.^2;
t38 = t24+t27+t35+8.1e+1./1.0e+3;
t37 = -t36;
t39 = 1.0./t38;
t40 = t33+t34+t37;
t41 = sqrt(t40);
t42 = t26+t28+t41;
t43 = t39.*t42;
t44 = atan(t43);
t45 = t44.*2.0;
t46 = -t45;
t47 = cos(t45);
t48 = sin(t45);
t49 = imag(t47);
t50 = real(t47);
t51 = imag(t48);
t52 = real(t48);
t53 = phi1+t46;
t55 = t47.*(2.7e+1./1.0e+2);
t56 = t48.*(2.7e+1./1.0e+2);
t62 = t48.*2.7e-1i;
t54 = sin(t53);
t57 = t49.*(2.7e+1./1.0e+2);
t58 = t50.*(2.7e+1./1.0e+2);
t59 = t51.*(2.7e+1./1.0e+2);
t60 = t52.*(2.7e+1./1.0e+2);
t63 = t12+t56;
t65 = t10+t55-3.0./4.0e+1;
t67 = t29+t56;
t69 = t10+t15+t55-3.0./2.0e+1;
t61 = -t58;
t64 = t63.^2;
t66 = t65.^2;
t68 = t67.^2;
t70 = 1.0./t69;
t72 = t17+t20+t57+t60;
t73 = t21+t62+t65;
t71 = t70.^2;
t74 = t19+t22+t59+t61+3.0./4.0e+1;
t75 = abs(t73);
t76 = t64+t66;
t79 = t67.*t70;
t77 = 1.0./t75;
t78 = sqrt(t76);
t80 = atan(t79);
t81 = t68.*t71;
t82 = t81+1.0;
t83 = -t80;
t87 = t46+t80;
t84 = phi4+t83;
t86 = 1.0./sqrt(t82);
t88 = sin(t87);
t85 = sin(t84);
t89 = 1.0./t88;
spd = [dphi1.*t54.*t89.*(t72.*t77.*t79.*t86+t74.*t77.*t78.*t86).*(-3.0./2.0e+1)+dphi4.*t85.*t89.*(t48.*t72.*t77+t47.*t74.*t77.*t78).*(3.0./2.0e+1);dphi1.*t54.*t89.*(t72.*t77.*t78.*t86-t74.*t77.*t79.*t86).*(-3.0./2.0e+1)-dphi4.*t85.*t89.*(t48.*t74.*t77-t47.*t72.*t77.*t78).*(3.0./2.0e+1)];
end
