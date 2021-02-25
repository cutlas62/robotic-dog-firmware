#include "leg.h"
#include <math.h>
#include <Arduino.h>

void Leg::moveFoot (int xCoor, int yCoor) {

}

// Same as moveFoot but just print the joint values
// instead of moving the feet, for testing purposes
void Leg::fakeMoveFoot (double x, double y) {
  double a = femurLength;
  double b = tibiaLength;
  double q2 = acos((x * x + y * y - a * a - b * b) / (2 * a * b));
  double q1 = atan2(y, x) - atan2((b * sin(q2)), (a + b * cos(q2)));
  Serial.print("[x,y,q1,q2] = [");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(q1);
  Serial.print(",");
  Serial.print(q2);
  Serial.println("]");
}
