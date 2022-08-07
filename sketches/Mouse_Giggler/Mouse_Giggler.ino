#include <Mouse.h>

/*
 *  NAME: Arduino Mouse Jiggler
 *  DATE: 2019-09-18
 *  DESC: Arduino based mouse emulator
 *  VERSION: 1.0
 */
 
int move_interval = 3;
int loop_interval = 60000;

void setup()
{
  Serial.begin(9600);
  randomSeed(analogRead(0));
  Mouse.begin();
}

void loop()
{
  int distance = random(10, 800);
  int x = random(3) - 1;
  int y = random(3) - 1;
  for (int i = 0; i < distance; i++) {
    Mouse.move(x, y, 0);
    delay(move_interval);
  }
  delay(loop_interval);
}