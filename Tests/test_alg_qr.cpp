//
// Created by Danila on 19.05.2019.
//

#include "test.h"
#include "../highLogic/alg.h"

TEST(Alg, 1) {
    Robot robot;
    do_alg_code(robot, true, "(M,F,N,H)(P,K,R,M)(F,H,D,J)(I,R,K,T)");
}
