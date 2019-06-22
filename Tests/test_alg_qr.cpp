//
// Created by Danila on 19.05.2019.
//

#include "test.h"
#include "../highLogic/alg.h"

TEST(Alg, 1) {
    Robot robot;
    //do_alg_code(robot, false, "(K,G,J,D)(F,H,D,J)(S,F,U,H)(I,Q,G,S)");
    do_alg_code(robot, false, "(F,J,H,H)(I,R,K,T)(O,A,M,C)(Q,Q,O,S)");
}
