#include "TuneFilterDecimate_base.h"

/*******************************************************************************************

    AUTO-GENERATED CODE. DO NOT MODIFY

    The following class functions are for the base class for the component class. To
    customize any of these functions, do not modify them here. Instead, overload them
    on the child class

******************************************************************************************/

TuneFilterDecimate_base::TuneFilterDecimate_base(const char *uuid, const char *label) :
    Component(uuid, label),
    ThreadedComponent()
{
    loadProperties();

    dataFloat_in = new bulkio::InFloatPort("dataFloat_in");
    addPort("dataFloat_in", dataFloat_in);
    dataFloat_out = new bulkio::OutFloatPort("dataFloat_out");
    addPort("dataFloat_out", dataFloat_out);
}

TuneFilterDecimate_base::~TuneFilterDecimate_base()
{
    delete dataFloat_in;
    dataFloat_in = 0;
    delete dataFloat_out;
    dataFloat_out = 0;
}

/*******************************************************************************************
    Framework-level functions
    These functions are generally called by the framework to perform housekeeping.
*******************************************************************************************/
void TuneFilterDecimate_base::start() throw (CORBA::SystemException, CF::Resource::StartError)
{
    Component::start();
    ThreadedComponent::startThread();
}

void TuneFilterDecimate_base::stop() throw (CORBA::SystemException, CF::Resource::StopError)
{
    Component::stop();
    if (!ThreadedComponent::stopThread()) {
        throw CF::Resource::StopError(CF::CF_NOTSET, "Processing thread did not die");
    }
}

void TuneFilterDecimate_base::releaseObject() throw (CORBA::SystemException, CF::LifeCycle::ReleaseError)
{
    // This function clears the component running condition so main shuts down everything
    try {
        stop();
    } catch (CF::Resource::StopError& ex) {
        // TODO - this should probably be logged instead of ignored
    }

    Component::releaseObject();
}

void TuneFilterDecimate_base::loadProperties()
{
    addProperty(TuneMode,
                "NORM",
                "TuneMode",
                "",
                "readwrite",
                "",
                "external",
                "execparam,configure");

    addProperty(TuningNorm,
                0.0,
                "TuningNorm",
                "",
                "readwrite",
                "",
                "external",
                "configure");

    addProperty(TuningIF,
                0,
                "TuningIF",
                "",
                "readwrite",
                "Hz",
                "external",
                "configure");

    addProperty(TuningRF,
                0LL,
                "TuningRF",
                "",
                "readwrite",
                "Hz",
                "external",
                "configure");

    addProperty(FilterBW,
                8000,
                "FilterBW",
                "",
                "readwrite",
                "Hz",
                "external",
                "configure");

    addProperty(DesiredOutputRate,
                10000,
                "DesiredOutputRate",
                "",
                "readwrite",
                "Hz",
                "external",
                "configure");

    addProperty(ActualOutputRate,
                "ActualOutputRate",
                "",
                "readonly",
                "",
                "external",
                "configure");

    addProperty(InputRF,
                0.0,
                "InputRF",
                "",
                "readonly",
                "Hz",
                "external",
                "configure");

    addProperty(InputRate,
                0.0,
                "InputRate",
                "",
                "readonly",
                "Hz",
                "external",
                "configure");

    addProperty(DecimationFactor,
                "DecimationFactor",
                "",
                "readonly",
                "",
                "external",
                "configure");

    addProperty(taps,
                "taps",
                "",
                "readonly",
                "",
                "external",
                "configure");

    addProperty(filterProps,
                filterProps_struct(),
                "filterProps",
                "",
                "readwrite",
                "",
                "external",
                "configure");

}


