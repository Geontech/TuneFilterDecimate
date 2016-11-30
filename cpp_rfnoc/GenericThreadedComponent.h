#ifndef GENERIC_THREADED_COMPONENT_H
#define GENERIC_THREADED_COMPONENT_H

#include <boost/function.hpp>
#include <ossie/ThreadedComponent.h>

/*
 * A class for creating a service function thread
 */
class GenericThreadedComponent : public ThreadedComponent
{
    typedef boost::function<int()> serviceFunction_t;

    public:
        GenericThreadedComponent(serviceFunction_t sf);

        virtual int serviceFunction();

        void start();
        bool stop();

    private:
        serviceFunction_t serviceFunctionMethod;
};

#endif
