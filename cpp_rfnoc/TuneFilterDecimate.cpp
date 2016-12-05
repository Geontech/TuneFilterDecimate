/**************************************************************************

    This is the component code. This file contains the child class where
    custom functionality can be added to the component. Custom
    functionality to the base class can be extended here. Access to
    the ports can also be done from this class

**************************************************************************/

#include "TuneFilterDecimate.h"

PREPARE_LOGGING(TuneFilterDecimate_i)

TuneFilterDecimate_i::TuneFilterDecimate_i(const char *uuid, const char *label) :
    TuneFilterDecimate_base(uuid, label),
    receivedSRI(false),
    //rxThread(NULL),
    spp(512)//,
    //txThread(NULL)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);
}

TuneFilterDecimate_i::~TuneFilterDecimate_i()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);
}

void TuneFilterDecimate_i::constructor()
{
    /***********************************************************************************
     This is the RH constructor. All properties are properly initialized before this function is called 
    ***********************************************************************************/
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);
}

// Service functions for RX and TX
int TuneFilterDecimate_i::rxServiceFunction()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    return NOOP;
}

int TuneFilterDecimate_i::txServiceFunction()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    return NOOP;
}

// Override start and stop
void TuneFilterDecimate_i::start() throw (CF::Resource::StartError, CORBA::SystemException)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

}

void TuneFilterDecimate_i::stop() throw (CF::Resource::StopError, CORBA::SystemException)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

}

/*
 * A method which allows a callback to be set for the block ID changing. This
 * callback should point back to the persona to alert it of the component's
 * block IDs
 */
void TuneFilterDecimate_i::setBlockIDCallback(blockIDCallback cb)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    this->blockIDChange = cb;
}

/*
 * A method which allows the persona to set this component as an RX streamer.
 * This means the component should retrieve the data from block and then send
 * it out as bulkio data.
 */
void TuneFilterDecimate_i::setRxStreamer(bool enable)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (enable) {
        // Don't create an RX stream if it already exists
        /*if (this->rxStream.get()) {
            LOG_DEBUG(TuneFilterDecimate_i, "Attempted to set RX streamer, but already streaming");
            return;
        }

        LOG_DEBUG(TuneFilterDecimate_i, "Attempting to set RX streamer");

        // Get the RX stream
        retrieveRxStream();

        // Create the receive buffer
        this->output.resize(10*spp);

        // Start continuous streaming immediately
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        stream_cmd.num_samps = 0;
        stream_cmd.stream_now = true;
        stream_cmd.time_spec = uhd::time_spec_t();

        this->rxStream->issue_stream_cmd(stream_cmd);

        // Create the RX receive thread
        this->rxThread = new GenericThreadedComponent(boost::bind(&TuneFilterDecimate_i::rxServiceFunction, this));

        // If the component is already started, then start the RX receive thread
        if (this->_started) {
            this->rxThread->start();
        }*/
    } else {
        // Don't clean up the stream if it's not already running
        /*if (not this->rxStream.get()) {
            LOG_DEBUG(TuneFilterDecimate_i, "Attempted to unset RX streamer, but not streaming");
            return;
        }

        // Stop continuous streaming
        uhd::stream_cmd_t streamCmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);

        this->rxStream->issue_stream_cmd(streamCmd);

        // Stop and delete the RX stream thread
        if (not this->rxThread->stop()) {
            LOG_WARN(TuneFilterDecimate_i, "RX Thread had to be killed");
        }

        // Release the RX stream pointer
        this->rxStream.reset();

        delete this->rxThread;
        this->rxThread = NULL;*/
    }
}

/*
 * A method which allows the persona to set this component as a TX streamer.
 * This means the component should retrieve the data from the bulkio port and
 *  then send it to the block.
 */
void TuneFilterDecimate_i::setTxStreamer(bool enable)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (enable) {
        // Don't create a TX stream if it already exists
        /*if (this->txStream.get()) {
            LOG_DEBUG(TuneFilterDecimate_i, "Attempted to set TX streamer, but already streaming");
            return;
        }

        LOG_DEBUG(TuneFilterDecimate_i, "Attempting to set TX streamer");

        // Get the TX stream
        retrieveTxStream();        // Avoid placing constructor code here. Instead, use the "constructor" function.


        // Create the TX transmit thread
        this->txThread = new GenericThreadedComponent(boost::bind(&TuneFilterDecimate_i::txServiceFunction, this));

        // If the component is already started, then start the TX transmit thread
        if (this->_started) {
            this->txThread->start();
        }*/
    } else {
        // Don't clean up the stream if it's not already running
        /*if (not this->txStream.get()) {
            LOG_DEBUG(TuneFilterDecimate_i, "Attempted to unset TX streamer, but not streaming");
            return;
        }

        // Stop and delete the TX stream thread
        if (not this->txThread->stop()) {
            LOG_WARN(TuneFilterDecimate_i, "TX Thread had to be killed");
        }

        // Release the TX stream pointer
        this->txStream.reset();

        delete this->txThread;
        this->txThread = NULL;*/
    }
}

/*
 * A method which allows the persona to set the address of the USRP it is
 * using.
 */
void TuneFilterDecimate_i::setUsrpAddress(uhd::device_addr_t usrpAddress)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Retrieve a pointer to the device
    /*this->usrp = uhd::device3::make(usrpAddress);

    // Save the address for later, if needed
    this->usrpAddress = usrpAddress;

    // Without a valid USRP, this component can't do anything
    if (not usrp.get()) {
        LOG_FATAL(TuneFilterDecimate_i, "Received a USRP which is not RF-NoC compatible.");
        throw std::exception();
    }*/
}

void TuneFilterDecimate_i::streamChanged(bulkio::InShortPort::StreamType stream)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);
}

void TuneFilterDecimate_i::retrieveRxStream()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Release the old stream if necessary
    /*if (this->rxStream.get()) {
        LOG_DEBUG(TuneFilterDecimate_i, "Releasing old RX stream");
        this->rxStream.reset();
    }

    // Set the stream arguments
    // Only support short complex for now
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::device_addr_t streamer_args;

    streamer_args["block_id"] = this->blockID;

    // Get the spp from the block
    this->spp = this->rfnocBlock->get_args().cast<size_t>("spp", 1024);

    streamer_args["spp"] = boost::lexical_cast<std::string>(this->spp);

    stream_args.args = streamer_args;

    LOG_DEBUG(TuneFilterDecimate_i, "Using streamer arguments: " << stream_args.args.to_string());

    // Retrieve the RX stream as specified from the device 3
    try {
        this->rxStream = this->usrp->get_rx_stream(stream_args);
    } catch(uhd::runtime_error &e) {
        LOG_ERROR(TuneFilterDecimate_i, "Failed to retrieve RX stream: " << e.what());
    } catch(...) {
        LOG_ERROR(TuneFilterDecimate_i, "Unexpected error occurred while retrieving RX stream");
    }*/
}

void TuneFilterDecimate_i::retrieveTxStream()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Release the old stream if necessary
    /*if (this->txStream.get()) {
        LOG_DEBUG(TuneFilterDecimate_i, "Releasing old TX stream");
        this->txStream.reset();
    }

    // Set the stream arguments
    // Only support short complex for now
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::device_addr_t streamer_args;

    streamer_args["block_id"] = this->blockID;

    // Get the spp from the block
    this->spp = this->rfnocBlock->get_args().cast<size_t>("spp", 1024);

    streamer_args["spp"] = boost::lexical_cast<std::string>(this->spp);

    stream_args.args = streamer_args;

    LOG_DEBUG(TuneFilterDecimate_i, "Using streamer arguments: " << stream_args.args.to_string());

    // Retrieve the TX stream as specified from the device 3
    try {
        this->txStream = this->usrp->get_tx_stream(stream_args);
    } catch(uhd::runtime_error &e) {
        LOG_ERROR(TuneFilterDecimate_i, "Failed to retrieve TX stream: " << e.what());
    } catch(...) {
        LOG_ERROR(TuneFilterDecimate_i, "Unexpected error occurred while retrieving TX stream");
    }*/
}

