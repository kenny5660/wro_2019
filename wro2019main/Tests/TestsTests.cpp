#include <gtest/gtest.h>
#include <stdio.h>

/*
	This is a very basic sample demonstrating the GoogleTest framework.
	Read more about CppUTest syntax here: https://github.com/google/googletest
*/



TEST(DemoTestGroup, SuccessfulTest1g)
{
	//This test should succeed
    EXPECT_EQ(1, 1);
}

TEST(DemoTestGroup, SuccessfulTest2g)
{
	//This test should succeed;
	printf("Hello from Test #2");
}
