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
    decimatorBlockId("0/KeepOneInN_0"),
    filterBlockId("0/FIR_0"),
    receivedSRI(false),
    rxThread(NULL),
    spp(512),
    txThread(NULL)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);
}

TuneFilterDecimate_i::~TuneFilterDecimate_i()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Stop streaming
    if (this->rxStream.get()) {
        uhd::stream_cmd_t streamCmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);

        this->rxStream->issue_stream_cmd(streamCmd);
    }

    // Reset the RF-NoC blocks
    if (this->decimator.get()) {
        this->decimator->clear();
    }

    if (this->filter.get()) {
        this->filter->clear();
    }

    // Release the threads if necessary
    if (this->rxThread) {
        delete this->rxThread;
    }

    if (this->txThread) {
        delete this->txThread;
    }

    LOG_INFO(TuneFilterDecimate_i, "END OF DESTRUCTOR");
}

void TuneFilterDecimate_i::constructor()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Grab the pointers to the blocks
    this->decimator = this->usrp->get_block_ctrl<uhd::rfnoc::block_ctrl_base>(this->decimatorBlockId);
    this->filter = this->usrp->get_block_ctrl<uhd::rfnoc::fir_block_ctrl>(this->filterBlockId);

    // Without either of these, there's no need to continue
    if (not this->decimator) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC block with ID: " << this->decimatorBlockId);
        throw std::exception();
    } else {
        LOG_DEBUG(TuneFilterDecimate_i, "Got the block: " << this->decimatorBlockId);
    }

    if (not this->filter) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC block with ID: " << this->filterBlockId);
        throw std::exception();
    } else {
        LOG_DEBUG(TuneFilterDecimate_i, "Got the block: " << this->filterBlockId);
    }

    // Setup based on properties initially


    // Register property change listeners


    // Alert the persona of the block IDs
    if (this->blockIDChange) {
        std::vector<uhd::rfnoc::block_id_t> blocks;

        blocks.push_back(this->decimatorBlockId);
        blocks.push_back(this->filterBlockId);

        this->blockIDChange(this->_identifier, blocks);
    }

    // Add an SRI change listener
    this->dataFloat_in->addStreamListener(this, &TuneFilterDecimate_i::streamChanged);

    // Preallocate the vectors
    //this->floatOutput.resize(10000);
    //this->output.resize(10000);
}

// Service functions for RX and TX
int TuneFilterDecimate_i::rxServiceFunction()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Perform RX, if necessary
    if (this->rxStream.get()) {
        // Don't bother doing anything until the SRI has been received
        if (not this->receivedSRI) {
            LOG_DEBUG(TuneFilterDecimate_i, "RX Thread active but no SRI has been received");
            return NOOP;
        }

        // Recv from the block
        uhd::rx_metadata_t md;

        LOG_DEBUG(TuneFilterDecimate_i, "Calling recv on the rx_stream");

        size_t num_rx_samps = this->rxStream->recv(&this->output.front(), this->output.size(), md, 3.0);

        // Check the meta data for error codes
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            LOG_ERROR(TuneFilterDecimate_i, "Timeout while streaming");
            return NOOP;
        } else if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            LOG_WARN(TuneFilterDecimate_i, "Overflow while streaming");
        } else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            LOG_WARN(TuneFilterDecimate_i, md.strerror());
            return NOOP;
        }

        LOG_DEBUG(TuneFilterDecimate_i, "RX Thread Requested " << this->output.size() << " samples");
        LOG_DEBUG(TuneFilterDecimate_i, "RX Thread Received " << num_rx_samps << " samples");

        // Get the time stamps from the meta data
        BULKIO::PrecisionUTCTime rxTime;

        rxTime.twsec = md.time_spec.get_full_secs();
        rxTime.tfsec = md.time_spec.get_frac_secs();

        // Convert the short data to float data
        //this->floatOutput.assign(this->output.begin(), this->output.end());
        this->floatOutput.resize(this->output.size());

        for (size_t i = 0; i < this->floatOutput.size(); ++i) {
            this->floatOutput[i].real((float) this->output[i].real());
            this->floatOutput[i].imag((float) this->output[i].imag());
        }

        // Write the data to the output stream
        float *outputBuffer = (float *) this->floatOutput.data();

        this->dataFloat_out->pushPacket(outputBuffer, this->floatOutput.size() * 2, rxTime, md.end_of_burst, this->sri.streamID._ptr);
    }

    return NORMAL;
}

int TuneFilterDecimate_i::txServiceFunction()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Perform TX, if necessary
    if (this->txStream.get()) {
        // Wait on input data
        bulkio::InFloatPort::DataTransferType *packet = this->dataFloat_in->getPacket(bulkio::Const::BLOCKING);

        if (not packet) {
            return NOOP;
        }

        // Convert the float data to short data
        this->shortInput.assign(packet->dataBuffer.begin(), packet->dataBuffer.end());

        uhd::tx_metadata_t md;
        std::complex<short> *block = (std::complex<short> *) this->shortInput.data();
        size_t blockSize = this->shortInput.size() / 2;

        LOG_DEBUG(TuneFilterDecimate_i, "TX Thread Received " << blockSize << " samples");

        if (blockSize == 0) {
            LOG_DEBUG(TuneFilterDecimate_i, "Skipping empty packet");
            delete packet;
            return NOOP;
        }

        // Get the timestamp to send to the RF-NoC block
        BULKIO::PrecisionUTCTime time = packet->T;

        md.has_time_spec = true;
        md.time_spec = uhd::time_spec_t(time.twsec, time.tfsec);

        // Send the data
        size_t num_tx_samps = this->txStream->send(block, blockSize, md);

        if (blockSize != 0 and num_tx_samps == 0) {
            LOG_DEBUG(TuneFilterDecimate_i, "The TX stream is no longer valid, obtaining a new one");

            retrieveTxStream();
        }

        LOG_DEBUG(TuneFilterDecimate_i, "TX Thread Sent " << num_tx_samps << " samples");

        // On EOS, forward to the RF-NoC Block
        if (packet->EOS) {
            LOG_DEBUG(TuneFilterDecimate_i, "EOS");

            md.end_of_burst = true;

            std::vector<std::complex<short> > empty;
            this->txStream->send(&empty.front(), empty.size(), md);
        }

        delete packet;
    }

    return NORMAL;
}

// Override start and stop
void TuneFilterDecimate_i::start() throw (CF::Resource::StartError, CORBA::SystemException)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    TuneFilterDecimate_base::start();

    if (this->rxThread) {
        this->rxThread->start();
    }

    if (this->txThread) {
        this->txThread->start();
    }
}

void TuneFilterDecimate_i::stop() throw (CF::Resource::StopError, CORBA::SystemException)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (this->rxThread) {
        if (not this->rxThread->stop()) {
            LOG_WARN(TuneFilterDecimate_i, "RX Thread had to be killed");
        }
    }

    if (this->txThread) {
        if (not this->txThread->stop()) {
            LOG_WARN(TuneFilterDecimate_i, "TX Thread had to be killed");
        }
    }

    TuneFilterDecimate_base::stop();
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
        if (this->rxStream.get()) {
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
        }
    } else {
        // Don't clean up the stream if it's not already running
        if (not this->rxStream.get()) {
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
        this->rxThread = NULL;
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
        if (this->txStream.get()) {
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
        }
    } else {
        // Don't clean up the stream if it's not already running
        if (not this->txStream.get()) {
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
        this->txThread = NULL;
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
    this->usrp = uhd::device3::make(usrpAddress);

    // Save the address for later, if needed
    this->usrpAddress = usrpAddress;

    // Without a valid USRP, this component can't do anything
    if (not usrp.get()) {
        LOG_FATAL(TuneFilterDecimate_i, "Received a USRP which is not RF-NoC compatible.");
        throw std::exception();
    }
}

void TuneFilterDecimate_i::streamChanged(bulkio::InFloatPort::StreamType stream)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    this->sri = stream.sri();

    // Default to complex
    this->sri.mode = 1;

    this->dataFloat_out->pushSRI(this->sri);

    this->receivedSRI = true;
}

void TuneFilterDecimate_i::retrieveRxStream()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Release the old stream if necessary
    if (this->rxStream.get()) {
        LOG_DEBUG(TuneFilterDecimate_i, "Releasing old RX stream");
        this->rxStream.reset();
    }

    // Set the stream arguments
    // Only support short complex for now
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::device_addr_t streamer_args;

    streamer_args["block_id"] = this->decimatorBlockId;

    // Get the spp from the block
    this->spp = this->decimator->get_args().cast<size_t>("spp", 1024);

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
    }
}

void TuneFilterDecimate_i::retrieveTxStream()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Release the old stream if necessary
    if (this->txStream.get()) {
        LOG_DEBUG(TuneFilterDecimate_i, "Releasing old TX stream");
        this->txStream.reset();
    }

    // Set the stream arguments
    // Only support short complex for now
    uhd::stream_args_t stream_args("sc16", "sc16");
    uhd::device_addr_t streamer_args;

    streamer_args["block_id"] = this->filterBlockId;

    // Get the spp from the block
    this->spp = this->filter->get_args().cast<size_t>("spp", 1024);

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
    }
}

