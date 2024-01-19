#include <stdlib.h>
#include <pico/binary_info/code.h>

#include "src/homer2_main.h"

int main() {
    bi_decl(bi_program_description("Homer2"));

    homer2_main();

    // We're expected to run forever.
    return EXIT_FAILURE;
}
