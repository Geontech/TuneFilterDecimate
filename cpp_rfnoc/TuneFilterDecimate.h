#ifndef TUNEFILTERDECIMATE_I_IMPL_H
#define TUNEFILTERDECIMATE_I_IMPL_H

#include "TuneFilterDecimate_base.h"

#include <uhd/rfnoc/block_ctrl.hpp>
#include <uhd/rfnoc/fir_block_ctrl.hpp>
#include <uhd/rfnoc/graph.hpp>
#include <uhd/device3.hpp>

#include "GenericThreadedComponent.h"
#include "RFNoC_Component.h"

class TuneFilterDecimate_i : public TuneFilterDecimate_base, public RFNoC_ComponentInterface
{
    ENABLE_LOGGING
    public:
        TuneFilterDecimate_i(const char *uuid, const char *label);
        ~TuneFilterDecimate_i();

        void constructor();

        // Service functions for RX and TX
        int rxServiceFunction();
        int txServiceFunction();

        // Don't use the default serviceFunction for clarity
        int serviceFunction() { return FINISH; }

        // Override start and stop
        void start() throw (CF::Resource::StartError, CORBA::SystemException);
        void stop() throw (CF::Resource::StopError, CORBA::SystemException);

        // Methods to be called by the persona, inherited from RFNoC_ComponentInterface
        void setBlockIDCallback(blockIDCallback cb);
        void setRxStreamer(bool enable);
        void setTxStreamer(bool enable);
        void setUsrpAddress(uhd::device_addr_t usrpAddress);

    private:
        // Stream listeners
        void streamChanged(bulkio::InShortPort::StreamType stream);

    private:
        void retrieveRxStream();
        void retrieveTxStream();

    private:
        blockIDCallback blockIDChange;
        uhd::rfnoc::fir_block_ctrl::sptr filter;
        std::vector<std::complex<short> > output;
        bool receivedSRI;
        uhd::rfnoc::block_ctrl_base::sptr rfnocBlock;
        uhd::rx_streamer::sptr rxStream;
        GenericThreadedComponent *rxThread;
        size_t spp;
        BULKIO::StreamSRI sri;
        uhd::tx_streamer::sptr txStream;
        GenericThreadedComponent *txThread;
        uhd::device3::sptr usrp;
        uhd::device_addr_t usrpAddress;
};

#endif // TUNEFILTERDECIMATE_I_IMPL_H
