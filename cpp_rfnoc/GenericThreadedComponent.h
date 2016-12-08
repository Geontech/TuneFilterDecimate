#ifndef GENERIC_THREADED_COMPONENT_H
#define GENERIC_THREADED_COMPONENT_H

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <ossie/debug.h>
#include <ossie/ThreadedComponent.h>

/*
 * A class for creating a service function thread
 */
class GenericThreadedComponent
{
    ENABLE_LOGGING

    typedef boost::function<int()> serviceFunction_t;

    public:
        GenericThreadedComponent(serviceFunction_t sf);
        ~GenericThreadedComponent();

        void serviceFunction();

        void start();
        bool stop();

        float getThreadDelay() const { return this->noopDelay; }
        void setThreadDelay(float delay) { this->noopDelay = delay; }

    private:
        float noopDelay;
        serviceFunction_t serviceFunctionMethod;
        boost::thread *thread;
};

#endif
