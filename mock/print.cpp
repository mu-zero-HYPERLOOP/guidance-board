#include <stdarg.h>
#include <cstdio>


void debugPrintf( const char* format, ... ) {
    va_list args;
    va_start( args, format );
    vprintf(format, args);
    va_end( args );
}

void debugPrintChar(char c){
  putchar(c);
}

void debugPrintFlush(){
  fflush(stdout);
}
