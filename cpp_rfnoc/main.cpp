#include <iostream>
#include "ossie/ossieSupport.h"

#include "TuneFilterDecimate.h"
int main(int argc, char* argv[])
{
    TuneFilterDecimate_i* TuneFilterDecimate_servant;
    Component::start_component(TuneFilterDecimate_servant, argc, argv);
    return 0;
}

