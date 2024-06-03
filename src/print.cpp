#include <stdarg.h>
#include <Arduino.h>

void debugPrintf( const char* format, ... ) {
    va_list args;
    va_start( args, format );
    Serial.vprintf(format, args);
    va_end( args );
}

void debugPrintChar(char c){
  Serial.print(c);
}

void debugPrintFlush(){
  Serial.flush();
}
