#include <cassert>
#include <cmath>
#include "Disk.hpp"
#include "Mobile.hpp"
#include "Point.hpp"

void test_IsCollided();
void test_escapeAngle();
void test_edgeProjection();
void TestDiskMethods();
void advance_until_contact_and_print(Mobile & m, Environment & E, double h, int max_steps, const std::string & tag);