#ifndef TUNEFILTERDECIMATE_I_IMPL_H
#define TUNEFILTERDECIMATE_I_IMPL_H

#include "TuneFilterDecimate_base.h"

#include <liquid/liquid.h>
#include <uhd/device3.hpp>
#include <uhd/rfnoc/block_ctrl.hpp>
#include <uhd/rfnoc/fir_block_ctrl.hpp>
#include <uhd/rfnoc/graph.hpp>

#include "GenericThreadedComponent.h"
#include "RFNoC_Component.h"

class TuneFilterDecimate_i : public TuneFilterDecimate_base, public RFNoC_ComponentInterface
{
    ENABLE_LOGGING
    public:
        TuneFilterDecimate_i(const char *uuid, const char *label);
        virtual ~TuneFilterDecimate_i();

        void constructor();

        // Service functions for RX and TX
        int rxServiceFunction();
        int txServiceFunction();

        // Don't use the default serviceFunction for clarity
        int serviceFunction() { return FINISH; }

        // Override start and stop
        void start() throw (CF::Resource::StartError, CORBA::SystemException);
        void stop() throw (CF::Resource::StopError, CORBA::SystemException);

        // Override releaseObject
        void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

        // Methods to be called by the persona, inherited from RFNoC_ComponentInterface
        void setBlockIDCallback(blockIDCallback cb);
        void setRxStreamer(bool enable);
        void setTxStreamer(bool enable);
        void setUsrp(uhd::device3::sptr usrp);

    private:
        // Property listeners
        void DesiredOutputRateChanged(const float &oldValue, const float &newValue);
        void FilterBWChanged(const float &oldValue, const float &newValue);
        void filterPropsChanged(const filterProps_struct &oldValue, const filterProps_struct &newValue);
        void TuningIFChanged(const double &oldValue, const double &newValue);
        void TuningNormChanged(const double &oldValue, const double &newValue);
        void TuningRFChanged(const CORBA::ULongLong &oldValue, const CORBA::ULongLong &newValue);

        // Stream listeners
        void streamChanged(bulkio::InFloatPort::StreamType stream);

    private:
        bool configureFD(bool sriChanged = false);
        void retrieveRxStream();
        void retrieveTxStream();
        void startRxStream();
        void stopRxStream();

    private:
        // RF-NoC Members
        uhd::rfnoc::block_ctrl_base::sptr decimator;
        const uhd::rfnoc::block_id_t decimatorBlockId;
        size_t decimatorSpp;
        uhd::rfnoc::fir_block_ctrl::sptr filter;
        const uhd::rfnoc::block_id_t filterBlockId;
        size_t filterSpp;
        uhd::rfnoc::graph::sptr graph;
        uhd::device3::sptr usrp;

        // UHD Members
        uhd::rx_streamer::sptr rxStream;
        uhd::tx_streamer::sptr txStream;

        // Miscellaneous
        blockIDCallback blockIDChange;
        std::vector<std::complex<float> > floatOutput;
        std::vector<std::complex<short> > output;
        bool receivedSRI;
        bool rxStreamStarted;
        GenericThreadedComponent *rxThread;
        boost::mutex rxThreadLock;
        std::vector<short> shortInput;
        BULKIO::StreamSRI sri;
        GenericThreadedComponent *txThread;
        boost::mutex txThreadLock;

};

#endif // TUNEFILTERDECIMATE_I_IMPL_H
