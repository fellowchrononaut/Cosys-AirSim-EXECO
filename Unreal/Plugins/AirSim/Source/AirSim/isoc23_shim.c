#include <stdlib.h>

// Forward new GLIBC C23 symbol to classic strtol on older systems (e.g., glibc 2.35)
long __isoc23_strtol(const char* nptr, char** endptr, int base) {
    return strtol(nptr, endptr, base);
}