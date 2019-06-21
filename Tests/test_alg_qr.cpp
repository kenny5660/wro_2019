//
// Created by Danila on 19.05.2019.
//

#include "test.h"
#include "../highLogic/alg.h"

TEST(Alg, 1) {
    Robot robot;
    do_alg_code(robot, false, "(O,I,M,J)(R,M,T,O)(F,L,D,N)(N,R,P,T)");
}
