//
// Created by Danila on 19.05.2019.
//

#include "test.h"
#include "../highLogic/alg.h"

TEST(Alg, 1) {
    Robot robot;
    do_alg_code(robot, true, "(I,G,K,H)(T,I,R,K)(H,P,F,R)(N,R,P,T)");
}
