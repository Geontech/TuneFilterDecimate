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
    dataShort_in = new bulkio::InShortPort("dataShort_in");
    addPort("dataShort_in", dataShort_in);
    dataFloat_out = new bulkio::OutFloatPort("dataFloat_out");
    addPort("dataFloat_out", dataFloat_out);
    dataShort_out = new bulkio::OutShortPort("dataShort_out");
    addPort("dataShort_out", dataShort_out);
}

TuneFilterDecimate_base::~TuneFilterDecimate_base()
{
    delete dataFloat_in;
    dataFloat_in = 0;
    delete dataShort_in;
    dataShort_in = 0;
    delete dataFloat_out;
    dataFloat_out = 0;
    delete dataShort_out;
    dataShort_out = 0;
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
                "property");

    addProperty(TuningNorm,
                0.0,
                "TuningNorm",
                "",
                "readwrite",
                "",
                "external",
                "property");

    addProperty(TuningIF,
                0,
                "TuningIF",
                "",
                "readwrite",
                "Hz",
                "external",
                "property");

    addProperty(TuningRF,
                0LL,
                "TuningRF",
                "",
                "readwrite",
                "Hz",
                "external",
                "property");

    addProperty(FilterBW,
                8000,
                "FilterBW",
                "",
                "readwrite",
                "Hz",
                "external",
                "property");

    addProperty(DesiredOutputRate,
                10000,
                "DesiredOutputRate",
                "",
                "readwrite",
                "Hz",
                "external",
                "property");

    addProperty(ActualOutputRate,
                "ActualOutputRate",
                "",
                "readonly",
                "",
                "external",
                "property");

    addProperty(InputRF,
                0.0,
                "InputRF",
                "",
                "readonly",
                "Hz",
                "external",
                "property");

    addProperty(InputRate,
                0.0,
                "InputRate",
                "",
                "readonly",
                "Hz",
                "external",
                "property");

    addProperty(DecimationFactor,
                "DecimationFactor",
                "",
                "readonly",
                "",
                "external",
                "property");

    addProperty(taps,
                "taps",
                "",
                "readonly",
                "",
                "external",
                "property");

    addProperty(filterProps,
                filterProps_struct(),
                "filterProps",
                "",
                "readwrite",
                "",
                "external",
                "property");

}


