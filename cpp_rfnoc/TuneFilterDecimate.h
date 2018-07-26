#ifndef TUNEFILTERDECIMATE_I_IMPL_H
#define TUNEFILTERDECIMATE_I_IMPL_H

// Base Include(s)
#include "TuneFilterDecimate_base.h"

// Liquid Include(s)
#include <liquid/liquid.h>

// RF-NoC RH Include(s)
#include <GenericThreadedComponent.h>
#include <RFNoC_Component.h>

// UHD Include(s)
#include <uhd/rfnoc/block_ctrl_base.hpp>
#include <uhd/rfnoc/fir_block_ctrl.hpp>

class TuneFilterDecimate_i : public TuneFilterDecimate_base, public RFNoC_RH::RFNoC_Component
{
    ENABLE_LOGGING

	// Constructor(s) and/or Destructor
    public:
        TuneFilterDecimate_i(const char *uuid, const char *label);
        virtual ~TuneFilterDecimate_i();

    // Public Method(s)
    public:
        void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

        void start() throw (CF::Resource::StartError, CORBA::SystemException);

        void stop() throw (CF::Resource::StopError, CORBA::SystemException);

    // Public RFNoC_Component Method(s)
    public:
        // Methods to be called by the persona, inherited from RFNoC_Component
		void setRxStreamer(uhd::rx_streamer::sptr rxStreamer);

		void setTxStreamer(uhd::tx_streamer::sptr txStreamer);

    // Protected Method(s)
    protected:
        void constructor();

		// Don't use the default serviceFunction for clarity
		int serviceFunction() { return FINISH; }

		// Service functions for RX and TX
		int rxServiceFunction();

		int txServiceFunction();

	// Private Method(s)
    private:
		bool configureFD(bool sriChanged = false);

        void DesiredOutputRateChanged(const float &oldValue, const float &newValue);

        void FilterBWChanged(const float &oldValue, const float &newValue);

        void filterPropsChanged(const filterProps_struct &oldValue, const filterProps_struct &newValue);

        void newConnection(const char *connectionID);

        void newDisconnection(const char *connectionID);

        void sriChanged(const BULKIO::StreamSRI &newSRI);

        void startRxStream();

        void stopRxStream();

        void streamChanged(bulkio::InShortPort::StreamType stream);

        void TuningIFChanged(const double &oldValue, const double &newValue);

        void TuningNormChanged(const double &oldValue, const double &newValue);

        void TuningRFChanged(const CORBA::ULongLong &oldValue, const CORBA::ULongLong &newValue);

    private:
        uhd::rfnoc::block_ctrl_base::sptr decimator;
		size_t decimatorPort;
		size_t decimatorSpp;
        bool eob;
        bool expectEob;
        uhd::rfnoc::fir_block_ctrl::sptr filter;
        size_t filterPort;
        size_t filterSpp;
        std::vector<std::complex<short> > output;
        bool receivedSRI;
        uhd::rx_streamer::sptr rxStreamer;
        bool rxStreamStarted;
        boost::shared_ptr<RFNoC_RH::GenericThreadedComponent> rxThread;
        BULKIO::StreamSRI sri;
        std::map<std::string, bool> streamMap;
        uhd::tx_streamer::sptr txStreamer;
        boost::shared_ptr<RFNoC_RH::GenericThreadedComponent> txThread;
};

#endif
