//
// Created by Danila on 19.05.2019.
//

#include "test.h"
#include "../highLogic/alg.h"

TEST(Alg, 1) {
    Robot robot;
    do_alg_code(robot, true, "(K,F,N,F)(R,N,T,P)(D,M,F,O)(J,R,H,T)");
}
