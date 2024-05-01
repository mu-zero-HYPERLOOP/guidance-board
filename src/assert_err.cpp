#include <Arduino.h>

/**
 * This function get's invoked if a assertion fails
 */
void __assert_func(const char *filename, int line, const char *assert_func,
                   const char *expr) {
  Serial.printf("ASSERTION FAULT");
  while (true) {
    delay(1000);
  }
}
