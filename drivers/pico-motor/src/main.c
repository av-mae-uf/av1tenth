#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"

#include "pico/stdlib.h"


int main () {
  stdio_init_all();
  
  while (1) {
    tight_loop_contents();
  }
}
