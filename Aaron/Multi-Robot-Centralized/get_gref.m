function gref = get_gref(m10x,m10y,m11x,m11y,m12x,m12y,m13x,m13y,m14x,m14y,m15x,m15y,m1x,m2x,m3x,m4x,m5x,m6x,m7x,m8x,m9x,m1y,m2y,m3y,m4y,m5y,m6y,m7y,m8y,m9y,p1x,p2x,p3x,p4x,p5x,p1y,p2y,p3y,p4y,p5y,th1,th2,th3,th4,th5)
%GET_GREF
%    GREF = GET_GREF(M10X,M10Y,M11X,M11Y,M12X,M12Y,M13X,M13Y,M14X,M14Y,M15X,M15Y,M1X,M2X,M3X,M4X,M5X,M6X,M7X,M8X,M9X,M1Y,M2Y,M3Y,M4Y,M5Y,M6Y,M7Y,M8Y,M9Y,P1X,P2X,P3X,P4X,P5X,P1Y,P2Y,P3Y,P4Y,P5Y,TH1,TH2,TH3,TH4,TH5)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    05-Jun-2023 19:50:07

t2 = cos(th1);
t3 = cos(th2);
t4 = cos(th3);
t5 = cos(th4);
t6 = cos(th5);
t7 = sin(th1);
t8 = sin(th2);
t9 = sin(th3);
t10 = sin(th4);
t11 = sin(th5);
t12 = -p1x;
t13 = -p2x;
t14 = -p3x;
t15 = -p4x;
t16 = -p5x;
t17 = -p1y;
t18 = -p2y;
t19 = -p3y;
t20 = -p4y;
t21 = -p5y;
t22 = m10x+t12;
t23 = m10x+t13;
t24 = m11x+t12;
t25 = m10x+t14;
t26 = m11x+t13;
t27 = m12x+t12;
t28 = m10x+t15;
t29 = m11x+t14;
t30 = m12x+t13;
t31 = m13x+t12;
t32 = m10x+t16;
t33 = m11x+t15;
t34 = m12x+t14;
t35 = m13x+t13;
t36 = m14x+t12;
t37 = m11x+t16;
t38 = m12x+t15;
t39 = m13x+t14;
t40 = m14x+t13;
t41 = m15x+t12;
t42 = m12x+t16;
t43 = m13x+t15;
t44 = m14x+t14;
t45 = m15x+t13;
t46 = m13x+t16;
t47 = m14x+t15;
t48 = m15x+t14;
t49 = m14x+t16;
t50 = m15x+t15;
t51 = m15x+t16;
t52 = m10y+t17;
t53 = m10y+t18;
t54 = m11y+t17;
t55 = m10y+t19;
t56 = m11y+t18;
t57 = m12y+t17;
t58 = m10y+t20;
t59 = m11y+t19;
t60 = m12y+t18;
t61 = m13y+t17;
t62 = m10y+t21;
t63 = m11y+t20;
t64 = m12y+t19;
t65 = m13y+t18;
t66 = m14y+t17;
t67 = m11y+t21;
t68 = m12y+t20;
t69 = m13y+t19;
t70 = m14y+t18;
t71 = m15y+t17;
t72 = m12y+t21;
t73 = m13y+t20;
t74 = m14y+t19;
t75 = m15y+t18;
t76 = m13y+t21;
t77 = m14y+t20;
t78 = m15y+t19;
t79 = m14y+t21;
t80 = m15y+t20;
t81 = m15y+t21;
t82 = m1x+t12;
t83 = m1x+t13;
t84 = m2x+t12;
t85 = m1x+t14;
t86 = m2x+t13;
t87 = m3x+t12;
t88 = m1x+t15;
t89 = m2x+t14;
t90 = m3x+t13;
t91 = m4x+t12;
t92 = m1x+t16;
t93 = m2x+t15;
t94 = m3x+t14;
t95 = m4x+t13;
t96 = m5x+t12;
t97 = m2x+t16;
t98 = m3x+t15;
t99 = m4x+t14;
t100 = m5x+t13;
t101 = m6x+t12;
t102 = m3x+t16;
t103 = m4x+t15;
t104 = m5x+t14;
t105 = m6x+t13;
t106 = m7x+t12;
t107 = m4x+t16;
t108 = m5x+t15;
t109 = m6x+t14;
t110 = m7x+t13;
t111 = m8x+t12;
t112 = m5x+t16;
t113 = m6x+t15;
t114 = m7x+t14;
t115 = m8x+t13;
t116 = m9x+t12;
t117 = m6x+t16;
t118 = m7x+t15;
t119 = m8x+t14;
t120 = m9x+t13;
t121 = m7x+t16;
t122 = m8x+t15;
t123 = m9x+t14;
t124 = m8x+t16;
t125 = m9x+t15;
t126 = m9x+t16;
t127 = m1y+t17;
t128 = m1y+t18;
t129 = m2y+t17;
t130 = m1y+t19;
t131 = m2y+t18;
t132 = m3y+t17;
t133 = m1y+t20;
t134 = m2y+t19;
t135 = m3y+t18;
t136 = m4y+t17;
t137 = m1y+t21;
t138 = m2y+t20;
t139 = m3y+t19;
t140 = m4y+t18;
t141 = m5y+t17;
t142 = m2y+t21;
t143 = m3y+t20;
t144 = m4y+t19;
t145 = m5y+t18;
t146 = m6y+t17;
t147 = m3y+t21;
t148 = m4y+t20;
t149 = m5y+t19;
t150 = m6y+t18;
t151 = m7y+t17;
t152 = m4y+t21;
t153 = m5y+t20;
t154 = m6y+t19;
t155 = m7y+t18;
t156 = m8y+t17;
t157 = m5y+t21;
t158 = m6y+t20;
t159 = m7y+t19;
t160 = m8y+t18;
t161 = m9y+t17;
t162 = m6y+t21;
t163 = m7y+t20;
t164 = m8y+t19;
t165 = m9y+t18;
t166 = m7y+t21;
t167 = m8y+t20;
t168 = m9y+t19;
t169 = m8y+t21;
t170 = m9y+t20;
t171 = m9y+t21;
mt1 = [sqrt(abs(t82).^2+abs(t127).^2);atan2(-t7.*t82+t2.*t127,t2.*t82+t7.*t127);sqrt(abs(t84).^2+abs(t129).^2);atan2(-t7.*t84+t2.*t129,t2.*t84+t7.*t129);sqrt(abs(t87).^2+abs(t132).^2);atan2(-t7.*t87+t2.*t132,t2.*t87+t7.*t132);sqrt(abs(t91).^2+abs(t136).^2);atan2(-t7.*t91+t2.*t136,t2.*t91+t7.*t136);sqrt(abs(t96).^2+abs(t141).^2);atan2(-t7.*t96+t2.*t141,t2.*t96+t7.*t141);sqrt(abs(t101).^2+abs(t146).^2)];
mt2 = [atan2(-t7.*t101+t2.*t146,t2.*t101+t7.*t146);sqrt(abs(t106).^2+abs(t151).^2);atan2(-t7.*t106+t2.*t151,t2.*t106+t7.*t151);sqrt(abs(t111).^2+abs(t156).^2);atan2(-t7.*t111+t2.*t156,t2.*t111+t7.*t156);sqrt(abs(t116).^2+abs(t161).^2);atan2(-t7.*t116+t2.*t161,t2.*t116+t7.*t161);sqrt(abs(t22).^2+abs(t52).^2);atan2(-t7.*t22+t2.*t52,t2.*t22+t7.*t52);sqrt(abs(t24).^2+abs(t54).^2);atan2(-t7.*t24+t2.*t54,t2.*t24+t7.*t54)];
mt3 = [sqrt(abs(t27).^2+abs(t57).^2);atan2(-t7.*t27+t2.*t57,t2.*t27+t7.*t57);sqrt(abs(t31).^2+abs(t61).^2);atan2(-t7.*t31+t2.*t61,t2.*t31+t7.*t61);sqrt(abs(t36).^2+abs(t66).^2);atan2(-t7.*t36+t2.*t66,t2.*t36+t7.*t66);sqrt(abs(t41).^2+abs(t71).^2);atan2(-t7.*t41+t2.*t71,t2.*t41+t7.*t71);sqrt(abs(t83).^2+abs(t128).^2);atan2(-t8.*t83+t3.*t128,t3.*t83+t8.*t128);sqrt(abs(t86).^2+abs(t131).^2)];
mt4 = [atan2(-t8.*t86+t3.*t131,t3.*t86+t8.*t131);sqrt(abs(t90).^2+abs(t135).^2);atan2(-t8.*t90+t3.*t135,t3.*t90+t8.*t135);sqrt(abs(t95).^2+abs(t140).^2);atan2(-t8.*t95+t3.*t140,t3.*t95+t8.*t140);sqrt(abs(t100).^2+abs(t145).^2);atan2(-t8.*t100+t3.*t145,t3.*t100+t8.*t145);sqrt(abs(t105).^2+abs(t150).^2);atan2(-t8.*t105+t3.*t150,t3.*t105+t8.*t150);sqrt(abs(t110).^2+abs(t155).^2);atan2(-t8.*t110+t3.*t155,t3.*t110+t8.*t155)];
mt5 = [sqrt(abs(t115).^2+abs(t160).^2);atan2(-t8.*t115+t3.*t160,t3.*t115+t8.*t160);sqrt(abs(t120).^2+abs(t165).^2);atan2(-t8.*t120+t3.*t165,t3.*t120+t8.*t165);sqrt(abs(t23).^2+abs(t53).^2);atan2(-t8.*t23+t3.*t53,t3.*t23+t8.*t53);sqrt(abs(t26).^2+abs(t56).^2);atan2(-t8.*t26+t3.*t56,t3.*t26+t8.*t56);sqrt(abs(t30).^2+abs(t60).^2);atan2(-t8.*t30+t3.*t60,t3.*t30+t8.*t60);sqrt(abs(t35).^2+abs(t65).^2)];
mt6 = [atan2(-t8.*t35+t3.*t65,t3.*t35+t8.*t65);sqrt(abs(t40).^2+abs(t70).^2);atan2(-t8.*t40+t3.*t70,t3.*t40+t8.*t70);sqrt(abs(t45).^2+abs(t75).^2);atan2(-t8.*t45+t3.*t75,t3.*t45+t8.*t75);sqrt(abs(t85).^2+abs(t130).^2);atan2(-t9.*t85+t4.*t130,t4.*t85+t9.*t130);sqrt(abs(t89).^2+abs(t134).^2);atan2(-t9.*t89+t4.*t134,t4.*t89+t9.*t134);sqrt(abs(t94).^2+abs(t139).^2);atan2(-t9.*t94+t4.*t139,t4.*t94+t9.*t139)];
mt7 = [sqrt(abs(t99).^2+abs(t144).^2);atan2(-t9.*t99+t4.*t144,t4.*t99+t9.*t144);sqrt(abs(t104).^2+abs(t149).^2);atan2(-t9.*t104+t4.*t149,t4.*t104+t9.*t149);sqrt(abs(t109).^2+abs(t154).^2);atan2(-t9.*t109+t4.*t154,t4.*t109+t9.*t154);sqrt(abs(t114).^2+abs(t159).^2);atan2(-t9.*t114+t4.*t159,t4.*t114+t9.*t159);sqrt(abs(t119).^2+abs(t164).^2);atan2(-t9.*t119+t4.*t164,t4.*t119+t9.*t164);sqrt(abs(t123).^2+abs(t168).^2)];
mt8 = [atan2(-t9.*t123+t4.*t168,t4.*t123+t9.*t168);sqrt(abs(t25).^2+abs(t55).^2);atan2(-t9.*t25+t4.*t55,t4.*t25+t9.*t55);sqrt(abs(t29).^2+abs(t59).^2);atan2(-t9.*t29+t4.*t59,t4.*t29+t9.*t59);sqrt(abs(t34).^2+abs(t64).^2);atan2(-t9.*t34+t4.*t64,t4.*t34+t9.*t64);sqrt(abs(t39).^2+abs(t69).^2);atan2(-t9.*t39+t4.*t69,t4.*t39+t9.*t69);sqrt(abs(t44).^2+abs(t74).^2);atan2(-t9.*t44+t4.*t74,t4.*t44+t9.*t74)];
mt9 = [sqrt(abs(t48).^2+abs(t78).^2);atan2(-t9.*t48+t4.*t78,t4.*t48+t9.*t78);sqrt(abs(t88).^2+abs(t133).^2);atan2(-t10.*t88+t5.*t133,t5.*t88+t10.*t133);sqrt(abs(t93).^2+abs(t138).^2);atan2(-t10.*t93+t5.*t138,t5.*t93+t10.*t138);sqrt(abs(t98).^2+abs(t143).^2);atan2(-t10.*t98+t5.*t143,t5.*t98+t10.*t143);sqrt(abs(t103).^2+abs(t148).^2);atan2(-t10.*t103+t5.*t148,t5.*t103+t10.*t148);sqrt(abs(t108).^2+abs(t153).^2)];
mt10 = [atan2(-t10.*t108+t5.*t153,t5.*t108+t10.*t153);sqrt(abs(t113).^2+abs(t158).^2);atan2(-t10.*t113+t5.*t158,t5.*t113+t10.*t158);sqrt(abs(t118).^2+abs(t163).^2);atan2(-t10.*t118+t5.*t163,t5.*t118+t10.*t163);sqrt(abs(t122).^2+abs(t167).^2);atan2(-t10.*t122+t5.*t167,t5.*t122+t10.*t167);sqrt(abs(t125).^2+abs(t170).^2);atan2(-t10.*t125+t5.*t170,t5.*t125+t10.*t170);sqrt(abs(t28).^2+abs(t58).^2);atan2(-t10.*t28+t5.*t58,t5.*t28+t10.*t58)];
mt11 = [sqrt(abs(t33).^2+abs(t63).^2);atan2(-t10.*t33+t5.*t63,t5.*t33+t10.*t63);sqrt(abs(t38).^2+abs(t68).^2);atan2(-t10.*t38+t5.*t68,t5.*t38+t10.*t68);sqrt(abs(t43).^2+abs(t73).^2);atan2(-t10.*t43+t5.*t73,t5.*t43+t10.*t73);sqrt(abs(t47).^2+abs(t77).^2);atan2(-t10.*t47+t5.*t77,t5.*t47+t10.*t77);sqrt(abs(t50).^2+abs(t80).^2);atan2(-t10.*t50+t5.*t80,t5.*t50+t10.*t80);sqrt(abs(t92).^2+abs(t137).^2)];
mt12 = [atan2(-t11.*t92+t6.*t137,t6.*t92+t11.*t137);sqrt(abs(t97).^2+abs(t142).^2);atan2(-t11.*t97+t6.*t142,t6.*t97+t11.*t142);sqrt(abs(t102).^2+abs(t147).^2);atan2(-t11.*t102+t6.*t147,t6.*t102+t11.*t147);sqrt(abs(t107).^2+abs(t152).^2);atan2(-t11.*t107+t6.*t152,t6.*t107+t11.*t152);sqrt(abs(t112).^2+abs(t157).^2);atan2(-t11.*t112+t6.*t157,t6.*t112+t11.*t157);sqrt(abs(t117).^2+abs(t162).^2);atan2(-t11.*t117+t6.*t162,t6.*t117+t11.*t162)];
mt13 = [sqrt(abs(t121).^2+abs(t166).^2);atan2(-t11.*t121+t6.*t166,t6.*t121+t11.*t166);sqrt(abs(t124).^2+abs(t169).^2);atan2(-t11.*t124+t6.*t169,t6.*t124+t11.*t169);sqrt(abs(t126).^2+abs(t171).^2);atan2(-t11.*t126+t6.*t171,t6.*t126+t11.*t171);sqrt(abs(t32).^2+abs(t62).^2);atan2(-t11.*t32+t6.*t62,t6.*t32+t11.*t62);sqrt(abs(t37).^2+abs(t67).^2);atan2(-t11.*t37+t6.*t67,t6.*t37+t11.*t67);sqrt(abs(t42).^2+abs(t72).^2)];
mt14 = [atan2(-t11.*t42+t6.*t72,t6.*t42+t11.*t72);sqrt(abs(t46).^2+abs(t76).^2);atan2(-t11.*t46+t6.*t76,t6.*t46+t11.*t76);sqrt(abs(t49).^2+abs(t79).^2);atan2(-t11.*t49+t6.*t79,t6.*t49+t11.*t79);sqrt(abs(t51).^2+abs(t81).^2);atan2(-t11.*t51+t6.*t81,t6.*t51+t11.*t81)];
gref = [mt1;mt2;mt3;mt4;mt5;mt6;mt7;mt8;mt9;mt10;mt11;mt12;mt13;mt14];
