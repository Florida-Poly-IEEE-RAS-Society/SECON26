#include "../include/util.h"

void main(void);

void Reset_Handler(void) {
    main();
    while (1);
}

void main(void) {
    // Initialize hardware here
    // Your embedded code
}
