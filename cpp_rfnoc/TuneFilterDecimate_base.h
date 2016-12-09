#ifndef TUNEFILTERDECIMATE_BASE_IMPL_BASE_H
#define TUNEFILTERDECIMATE_BASE_IMPL_BASE_H

#include <boost/thread.hpp>
#include <ossie/Component.h>
#include <ossie/ThreadedComponent.h>

#include <bulkio/bulkio.h>
#include "struct_props.h"

class TuneFilterDecimate_base : public Component, protected ThreadedComponent
{
    public:
        TuneFilterDecimate_base(const char *uuid, const char *label);
        virtual ~TuneFilterDecimate_base();

        void start() throw (CF::Resource::StartError, CORBA::SystemException);

        void stop() throw (CF::Resource::StopError, CORBA::SystemException);

        void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

        void loadProperties();

    protected:
        // Member variables exposed as properties
        /// Property: TuneMode
        std::string TuneMode;
        /// Property: TuningNorm
        double TuningNorm;
        /// Property: TuningIF
        double TuningIF;
        /// Property: TuningRF
        CORBA::ULongLong TuningRF;
        /// Property: FilterBW
        float FilterBW;
        /// Property: DesiredOutputRate
        float DesiredOutputRate;
        /// Property: ActualOutputRate
        double ActualOutputRate;
        /// Property: InputRF
        double InputRF;
        /// Property: InputRate
        double InputRate;
        /// Property: DecimationFactor
        CORBA::ULong DecimationFactor;
        /// Property: taps
        CORBA::ULong taps;
        /// Property: filterProps
        filterProps_struct filterProps;

        // Ports
        /// Port: dataFloat_in
        bulkio::InFloatPort *dataFloat_in;
        /// Port: dataFloat_out
        bulkio::OutFloatPort *dataFloat_out;

    private:
};
#endif // TUNEFILTERDECIMATE_BASE_IMPL_BASE_H
