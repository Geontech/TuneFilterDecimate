#include "GenericThreadedComponent.h"

/*
 * Initialize the service function callback
 */
GenericThreadedComponent::GenericThreadedComponent(serviceFunction_t sf) :
    serviceFunctionMethod(sf)
{
}

/*
 * Call the service function callback
 */
int GenericThreadedComponent::serviceFunction()
{
    return this->serviceFunctionMethod();
}

/*
 * Start the thread
 */
void GenericThreadedComponent::start()
{
    this->startThread();
}

/*
 * Stop the thread
 */
bool GenericThreadedComponent::stop()
{
    return this->stopThread();
}
