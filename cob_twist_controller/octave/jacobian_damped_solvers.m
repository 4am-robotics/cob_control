1; # prevent octave from thinking m-file
# Functions
clear all;
close all;

# Gewichtungen
k = -1;
W_4_4 = k * k * eye(4, 4);
W_3_3 = k * k * eye(3, 3);

EPS_4_4 = ones(4,4) * 1e-12;
EPS_3_3 = ones(3,3) * 1e-12;
EPS_3_4 = ones(3,4) * 1e-12;
EPS_4_3 = ones(4,3) * 1e-12;

function evalLeftRightPseudoInv(J, W_left, W_right, EPS)
    rank_J = rank(J)
    J_T = J';
    # Solution1: Control of redundant Robot Manipulators pseudoleft
    J_pseudoleft = inv(J_T * J + W_left) * J_T
    # Solution2: Modelling and Control of Robot Manipulators pseudoright
    J_pseudoright = J_T * inv(J * J_T + W_right)

    if ((J_pseudoleft - EPS) <= J_pseudoright && J_pseudoright <= (J_pseudoleft + EPS))
        disp("--> TRUE: Both solution are equal")
    else
        disp("--> XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX FALSE: Both solution are NOT equal")
    endif
endfunction

####################### START Zeilen > Spalten
disp("**********************************************************************")
disp("*******************")
disp("=> Zeilen > Spalten")
disp("*******************")
J = [1 -2 3;
     4 -5 6;
     7 8 -9;
     -10 11 12]
evalLeftRightPseudoInv(J, W_3_3, W_4_4, EPS_3_4)

####################### END Zeilen > Spalten

####################### START Zeilen == Spalten
disp("**********************************************************************")
disp("********************")
disp("=> Zeilen == Spalten")
disp("********************")
#J = [1 -2 3;
#     4 5 -6;
#     -7 8 9]
J = [-1 0 0;
     0 -1 0;
     0 0 -1]
evalLeftRightPseudoInv(J, W_3_3, W_3_3, EPS_3_3)

####################### END Zeilen == Spalten

####################### START Spalten > Zeilen
disp("**********************************************************************")
disp("*******************")
disp("=> Spalten > Zeilen")
disp("*******************")
J = [1  2  -3  4;
     5  -6  7  8;
     9 10 11 -12]
evalLeftRightPseudoInv(J, W_4_4, W_3_3, EPS_4_3)

####################### END Spalten > Zeilen


####################### START Spalten > Zeilen - lin. unabhängig
disp("**********************************************************************")
disp("*******************")
disp("=> Spalten > Zeilen - lin. abhängige Spalten")
disp("*******************")
J = [1  2  3    4;
     2  4  6    8;
     4  8 12   16]
evalLeftRightPseudoInv(J, W_4_4, W_3_3, EPS_4_3)

####################### END Spalten > Zeilen - lin. abhängig

####################### START Spalten < Zeilen - lin. abhängig
disp("**********************************************************************")
disp("*******************")
disp("=> Spalten < Zeilen - lin. abhängige Zeilen")
disp("*******************")
J = [ 1  2  -3;
     -2 -4   6;
      4  8  12;
      8 16  24;]
evalLeftRightPseudoInv(J, W_3_3, W_4_4, EPS_3_4)

####################### END Spalten < Zeilen - lin. abhängig


####################### START Spalten < Zeilen - lin. abhängig
disp("**********************************************************************")
disp("*******************")
disp("=> Spalten < Zeilen - Null Zeilen")
disp("*******************")
J = [ 0  0   0;
      0  0   0;
      0  0   0;
      0  0   0;]
evalLeftRightPseudoInv(J, W_3_3, W_4_4, EPS_3_4)

####################### END Spalten < Zeilen - lin. abhängig

####################### START Spalten > Zeilen - lin. unabhängig
disp("**********************************************************************")
disp("*******************")
disp("=> Spalten > Zeilen - Null Spalten")
disp("*******************")
J = [1  0  3    4;
     2  0  6    8;
     4  0 12   16]
evalLeftRightPseudoInv(J, W_4_4, W_3_3, EPS_4_3)

####################### END Spalten > Zeilen - lin. abhängig
