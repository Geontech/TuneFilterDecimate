// Component Include
#include "TuneFilterDecimate.h"

// OSSIE Include(s)
#include <ossie/Device_impl.h>

// RF-NoC RH Include(s)
#include <RFNoC_Persona.h>

TuneFilterDecimate_i *resourcePtr;

void signal_catcher(int sig)
{
    // IMPORTANT Don't call exit(...) in this function
    // issue all CORBA calls that you need for cleanup here before calling ORB shutdown
    if (resourcePtr) {
        resourcePtr->halt();
    }
}
int main(int argc, char* argv[])
{
    struct sigaction sa;
    sa.sa_handler = signal_catcher;
    sa.sa_flags = 0;
    resourcePtr = 0;

    //Component::start_component(&resourcePtr, argc, argv);
    Resource_impl::start_component(resourcePtr, argc, argv);
    return 0;
}

extern "C" {
Resource_impl* construct(int argc, char* argv[], RFNoC_RH::RFNoC_Persona *persona) {

    struct sigaction sa;
    sa.sa_handler = signal_catcher;
    sa.sa_flags = 0;
    resourcePtr = 0;

    Resource_impl::start_component(resourcePtr, argc, argv);

    // Any addition parameters passed into construct can now be
    // set directly onto resourcePtr since it is the instantiated
    // Redhawk device
    RFNoC_RH::RFNoC_Component *rfNocComponentPtr = dynamic_cast<RFNoC_RH::RFNoC_Component *>(resourcePtr);

    if (not rfNocComponentPtr)
    {
    	delete resourcePtr;
    	return NULL;
    }

    rfNocComponentPtr->setPersona(persona);

    return resourcePtr;
}
}
