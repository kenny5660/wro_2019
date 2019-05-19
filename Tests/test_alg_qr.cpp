//
// Created by Danila on 19.05.2019.
//

#include "test.h"
#include "../highLogic/alg.h"

TEST(Alg, 1) {
    Robot robot;
    do_alg_code(robot, true, "(J,I,I,G)(H,R,J,T)(Q,B,O,D)(F,M,D,O)");
}
