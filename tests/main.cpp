#define CATCH_CONFIG_RUNNER

#include "catch.hpp"
#include <Astra/Astra.h>

int main( int argc, char* const argv[] )
{
    astra::Astra::initialize();

    int result = 1;

    astra::Astra::terminate();

    return result;
}
