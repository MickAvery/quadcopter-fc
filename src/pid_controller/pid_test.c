#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "pid.h"

int main(int argc, char**argv)
{
  if( argc != 3 ) {
    fprintf(stderr, "Usage: ./test <desired_output> <actual_output>\n");
    exit(1);
  }

  int32_t desired_output = strtol(argv[1], NULL, 10);
  int32_t actual_output = strtol(argv[2], NULL, 10);
  int32_t correction;

  printf("Desired output = %d\nActual output = %d\n\n", desired_output, actual_output);

  while(1) {
    correction = pid_algorithm( desired_output, actual_output );
    printf("Calculated correction = %d\n", correction);
    actual_output += correction;
    sleep(2);
  }

  return 0;
}
