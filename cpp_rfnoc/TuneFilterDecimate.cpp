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
    decimatorSpp(512),
    filterBlockId("0/FIR_0"),
    filterSpp(512),
    receivedSRI(false),
    rxStreamStarted(false),
    rxThread(NULL),
    txThread(NULL)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);
}

TuneFilterDecimate_i::~TuneFilterDecimate_i()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Stop streaming
    stopRxStream();

    // Reset the RF-NoC blocks
    if (this->decimator.get()) {
        this->decimator->clear();
    }

    if (this->filter.get()) {
        this->filter->clear();
    }

    // Release the threads if necessary
    if (this->rxThread) {
        this->rxThread->stop();
        delete this->rxThread;
    }

    if (this->txThread) {
        this->txThread->stop();
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
    if (not this->decimator.get()) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC block with ID: " << this->decimatorBlockId);
        throw CF::LifeCycle::InitializeError();
    } else {
        LOG_DEBUG(TuneFilterDecimate_i, "Got the block: " << this->decimatorBlockId);
    }

    if (not this->filter.get()) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC block with ID: " << this->filterBlockId);
        throw CF::LifeCycle::InitializeError();
    } else {
        LOG_DEBUG(TuneFilterDecimate_i, "Got the block: " << this->filterBlockId);
    }

    // Create a graph
    this->graph = this->usrp->create_graph("TuneFilterDecimate_" + this->_identifier);

    // Without this, there's no need to continue
    if (not this->graph.get()) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC graph");
        throw CF::LifeCycle::InitializeError();
    }

    // Connect the blocks
    this->graph->connect(this->filterBlockId, this->decimatorBlockId);

    // Setup based on properties initially


    // Register property change listeners
    addPropertyListener(DesiredOutputRate, this, &TuneFilterDecimate_i::DesiredOutputRateChanged); //configureFilter
    addPropertyListener(FilterBW, this, &TuneFilterDecimate_i::FilterBWChanged); //configureFilter
    addPropertyListener(filterProps, this, &TuneFilterDecimate_i::filterPropsChanged); //configureFilter
    addPropertyListener(TuningIF, this, &TuneFilterDecimate_i::TuningIFChanged); //configureTuner
    addPropertyListener(TuningNorm, this, &TuneFilterDecimate_i::TuningNormChanged); //configureTuner
    addPropertyListener(TuningRF, this, &TuneFilterDecimate_i::TuningRFChanged); //configureTuner

    // Alert the persona of the block IDs
    if (this->blockIDChange) {
        std::vector<uhd::rfnoc::block_id_t> blocks;

        blocks.push_back(this->decimatorBlockId);
        blocks.push_back(this->filterBlockId);

        this->blockIDChange(this->_identifier, blocks);
    }

    // Add an SRI change listener
    this->dataShort_in->addStreamListener(this, &TuneFilterDecimate_i::streamChanged);

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

        LOG_TRACE(TuneFilterDecimate_i, "Calling recv on the rx_stream");

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

        LOG_TRACE(TuneFilterDecimate_i, "RX Thread Requested " << this->output.size() << " samples");
        LOG_TRACE(TuneFilterDecimate_i, "RX Thread Received " << num_rx_samps << " samples");

        // Get the time stamps from the meta data
        BULKIO::PrecisionUTCTime rxTime;

        rxTime.twsec = md.time_spec.get_full_secs();
        rxTime.tfsec = md.time_spec.get_frac_secs();

        // Write the data to the output stream
        this->dataShort_out->pushPacket((short *) this->output.data(), this->output.size() * 2, rxTime, md.end_of_burst, this->sri.streamID._ptr);
    }

    return NORMAL;
}

int TuneFilterDecimate_i::txServiceFunction()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Perform TX, if necessary
    if (this->txStream.get()) {
        // Wait on input data
        bulkio::InShortPort::DataTransferType *packet = this->dataShort_in->getPacket(bulkio::Const::BLOCKING);

        if (not packet) {
            return NOOP;
        }

        // Respond to the SRI changing
        if (packet->sriChanged) {
            sriChanged(packet->SRI);
        }

        // Prepare the metadata
        uhd::tx_metadata_t md;
        std::complex<short> *block = (std::complex<short> *) packet->dataBuffer.data();
        size_t blockSize = packet->dataBuffer.size() / 2;

        LOG_TRACE(TuneFilterDecimate_i, "TX Thread Received " << blockSize << " samples");

        if (blockSize == 0) {
            LOG_TRACE(TuneFilterDecimate_i, "Skipping empty packet");
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

        LOG_TRACE(TuneFilterDecimate_i, "TX Thread Sent " << num_tx_samps << " samples");

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
        startRxStream();

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

        stopRxStream();
    }

    if (this->txThread) {
        if (not this->txThread->stop()) {
            LOG_WARN(TuneFilterDecimate_i, "TX Thread had to be killed");
        }
    }

    TuneFilterDecimate_base::stop();
}

void TuneFilterDecimate_i::releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // This function clears the component running condition so main shuts down everything
    try {
        stop();
    } catch (CF::Resource::StopError& ex) {
        // TODO - this should probably be logged instead of ignored
    }

    releasePorts();
    stopPropertyChangeMonitor();
    // This is a singleton shared by all code running in this process
    //redhawk::events::Manager::Terminate();
    PortableServer::POA_ptr root_poa = ossie::corba::RootPOA();
    PortableServer::ObjectId_var oid = root_poa->servant_to_id(this);
    root_poa->deactivate_object(oid);

    component_running.signal();
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
        this->output.resize(10*decimatorSpp);

        // Create the RX receive thread
        this->rxThread = new GenericThreadedComponent(boost::bind(&TuneFilterDecimate_i::rxServiceFunction, this));

        // If the component is already started, then start the RX receive thread
        if (this->_started) {
            startRxStream();

            this->rxThread->start();
        }
    } else {
        // Don't clean up the stream if it's not already running
        if (not this->rxStream.get()) {
            LOG_DEBUG(TuneFilterDecimate_i, "Attempted to unset RX streamer, but not streaming");
            return;
        }

        // Stop and delete the RX stream thread
        if (not this->rxThread->stop()) {
            LOG_WARN(TuneFilterDecimate_i, "RX Thread had to be killed");
        }

        // Stop continuous streaming
        stopRxStream();

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
        retrieveTxStream();

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
 * A method which allows the persona to set the USRP it is using.
 */
void TuneFilterDecimate_i::setUsrp(uhd::device3::sptr usrp)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Save the USRP for later
    this->usrp = usrp;

    // Without a valid USRP, this component can't do anything
    if (not usrp.get()) {
        LOG_FATAL(TuneFilterDecimate_i, "Received a USRP which is not RF-NoC compatible.");
        throw std::exception();
    }
}

void TuneFilterDecimate_i::DesiredOutputRateChanged(const float &oldValue, const float &newValue)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    /*if (not this->receivedSRI) {
        LOG_WARN(TuneFilterDecimate_i, "Can't design filter without input SRI");
        this->DesiredOutputRate = oldValue;
        return;
    }

    if (this->InputRate < newValue) {
        LOG_ERROR(TuneFilterDecimate_i, "Attempted to set DesiredOutputRate to a value greater than the input sample rate of " << this->InputRate << " Sps");
        this->DesiredOutputRate = oldValue;
        return;
    }*/

    if (this->receivedSRI) {
        if (not configureFD()) {
            LOG_ERROR(TuneFilterDecimate_i, "Unable to configure filter/decimator with requested DesiredOutputRate");
            this->DesiredOutputRate = oldValue;
        } else {
            this->dataShort_out->pushSRI(this->sri);
        }
    } else {
        LOG_WARN(TuneFilterDecimate_i, "Not designing filter until receiving SRI");
    }
}

void TuneFilterDecimate_i::FilterBWChanged(const float &oldValue, const float &newValue)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    /*if (not this->receivedSRI) {
        LOG_WARN(TuneFilterDecimate_i, "Can't design filter without input SRI");
        this->FilterBW = oldValue;
        return;
    }*/

    if (newValue < 0.0) {
        LOG_WARN(TuneFilterDecimate_i, "Attempted to set FilterBW to a value less than zero");
        this->FilterBW = oldValue;
        return;
    }

    if (this->receivedSRI) {
        if (not configureFD()) {
            LOG_ERROR(TuneFilterDecimate_i, "Unable to configure filter/decimator with requested FilterBW");
            this->FilterBW = oldValue;
        }
    } else {
        LOG_WARN(TuneFilterDecimate_i, "Not designing filter until receiving SRI");
    }
}

void TuneFilterDecimate_i::filterPropsChanged(const filterProps_struct &oldValue, const filterProps_struct &newValue)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    /*if (not this->receivedSRI) {
        LOG_WARN(TuneFilterDecimate_i, "Can't design filter without input SRI");
        this->filterProps = oldValue;
        return;
    }*/

    if (newValue.Ripple < 0.0) {
        LOG_WARN(TuneFilterDecimate_i, "Attempted to set filterProps.Ripple to a value less than zero");
        this->filterProps = oldValue;
        return;
    }

    if (newValue.TransitionWidth < 0.0) {
        LOG_WARN(TuneFilterDecimate_i, "Attempted to set filterProps.TransitionWidth to a value less than zero");
        this->filterProps = oldValue;
        return;
    }

    if (this->receivedSRI) {
        if (not configureFD()) {
            LOG_ERROR(TuneFilterDecimate_i, "Unable to configure filter/decimator with requested filterProps");
            this->filterProps = oldValue;
        }
    } else {
        LOG_WARN(TuneFilterDecimate_i, "Not designing filter until receiving SRI");
    }
}

void TuneFilterDecimate_i::TuningIFChanged(const double &oldValue, const double &newValue)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    LOG_WARN(TuneFilterDecimate_i, "TUNING NOT YET IMPLEMENTED");
}

void TuneFilterDecimate_i::TuningNormChanged(const double &oldValue, const double &newValue)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    LOG_WARN(TuneFilterDecimate_i, "TUNING NOT YET IMPLEMENTED");
}

void TuneFilterDecimate_i::TuningRFChanged(const CORBA::ULongLong &oldValue, const CORBA::ULongLong &newValue)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    LOG_WARN(TuneFilterDecimate_i, "TUNING NOT YET IMPLEMENTED");
}

void TuneFilterDecimate_i::streamChanged(bulkio::InShortPort::StreamType stream)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    sriChanged(stream.sri());
}

bool TuneFilterDecimate_i::configureFD(bool sriChanged)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    LOG_DEBUG(TuneFilterDecimate_i, "ActualOutputRate: " << this->ActualOutputRate);
    LOG_DEBUG(TuneFilterDecimate_i, "DecimationFactor: " << this->DecimationFactor);
    LOG_DEBUG(TuneFilterDecimate_i, "DesiredOutputRate: " << this->DesiredOutputRate);
    LOG_DEBUG(TuneFilterDecimate_i, "FilterBW: " << this->FilterBW);
    LOG_DEBUG(TuneFilterDecimate_i, "InputRate: " << this->InputRate);

    float newDecimationFactor = floor(this->InputRate / this->DesiredOutputRate);
    float newActualOutputRate = this->InputRate / newDecimationFactor;

    // These must change when the SRI does
    if (sriChanged) {
        this->ActualOutputRate = newActualOutputRate;
        this->DecimationFactor = newDecimationFactor;
    }

    LOG_DEBUG(TuneFilterDecimate_i, "New Decimation Factor: " << newDecimationFactor);

    this->sri.xdelta = 1.0 / newActualOutputRate;

    if (newActualOutputRate < this->FilterBW) {
        LOG_WARN(TuneFilterDecimate_i, "ActualOutputRate " << newActualOutputRate << " is less than FilterBW " << this->FilterBW);
        return false;
    }

    LOG_DEBUG(TuneFilterDecimate_i, "New Actual Output Rate: " << newActualOutputRate);

    // Convert these to the format required by liquid
    float convertedRipple = -20 * log10(this->filterProps.Ripple);
    float convertedTransitionWidth = this->filterProps.TransitionWidth / this->InputRate;

    LOG_DEBUG(TuneFilterDecimate_i, "Ripple (dB): " << convertedRipple);
    LOG_DEBUG(TuneFilterDecimate_i, "Transition Width (Normalized): " << convertedTransitionWidth);

    size_t availableFilterLength = this->filter->get_n_taps();
    /*size_t estimatedFilterLength = estimate_req_filter_len(convertedTransitionWidth, convertedRipple);

    LOG_DEBUG(TuneFilterDecimate_i, "Available Filter Taps: " << availableFilterLength);
    LOG_DEBUG(TuneFilterDecimate_i, "Estimated Filter Taps for design: " << estimatedFilterLength);

    if (availableFilterLength < estimatedFilterLength) {
        LOG_WARN(TuneFilterDecimate_i, "Unable to satisfy transition width and/or ripple requirements. Estimated filter length: " << estimatedFilterLength << ", Available filter length: " << availableFilterLength);
        LOG_INFO(TuneFilterDecimate_i, "Attempting to adjust ripple");

        float newRipple = estimate_req_filter_As(convertedTransitionWidth, availableFilterLength);

        LOG_DEBUG(TuneFilterDecimate_i, "Trying ripple: " << pow(10, -newRipple / 20));

        estimatedFilterLength = estimate_req_filter_len(convertedTransitionWidth, newRipple);

        if (availableFilterLength < estimatedFilterLength) {
            LOG_WARN(TuneFilterDecimate_i, "Unable to satisfy transition width and/or ripple requirements. Estimated filter length: " << estimatedFilterLength << ", Available filter length: " << availableFilterLength);
            LOG_INFO(TuneFilterDecimate_i, "Attempting to adjust transition width");

            float newTransitionWidth = estimate_req_filter_df(newRipple, availableFilterLength);

            LOG_DEBUG(TuneFilterDecimate_i, "Trying transition width: " << newTransitionWidth * this->InputRate);

            estimatedFilterLength = estimate_req_filter_len(newTransitionWidth, newRipple);

            if (availableFilterLength < estimatedFilterLength) {
                LOG_ERROR(TuneFilterDecimate_i, "Unable to design filter");
                return false;
            }

            LOG_INFO(TuneFilterDecimate_i, "Adjusting filterProps.TransitionWidth property to match available design");
            this->filterProps.TransitionWidth = newTransitionWidth * this->InputRate;
        }

        LOG_INFO(TuneFilterDecimate_i, "Adjusting filterProps.Ripple property to match available design");
        this->filterProps.Ripple = pow(10, -newRipple / 20);
        return false;
    }*/

    float cutoff = this->FilterBW / (this->InputRate);

    LOG_DEBUG(TuneFilterDecimate_i, "Normalized Cutoff: " << cutoff);

    std::vector<float> filterTaps(availableFilterLength);

    LOG_DEBUG(TuneFilterDecimate_i, "Calculating filter taps...");

    float bands[4] = {
            0.0f,                               cutoff,
            cutoff + convertedTransitionWidth,  0.5
    };

    float des[2] = { 1.0, (float) this->filterProps.Ripple };

    float weights[2] = { 1.0, 1.0 };

    liquid_firdespm_btype btype = LIQUID_FIRDESPM_BANDPASS;
    liquid_firdespm_wtype wtype[2] = { LIQUID_FIRDESPM_FLATWEIGHT, LIQUID_FIRDESPM_FLATWEIGHT };

    firdespm_run(availableFilterLength, 2, bands, des, weights, wtype, btype, filterTaps.data());
    //liquid_firdes_kaiser(estimatedFilterLength, cutoff, convertedRipple, 0, filterTaps.data());

    LOG_DEBUG(TuneFilterDecimate_i, "Converting floating point taps to integer taps...");

    // Convert the taps to long
    std::vector<int> longFilterTaps(availableFilterLength);

    for (size_t i = 0; i < availableFilterLength; ++i) {
        longFilterTaps[i] = (pow(2, (sizeof(short int) * 8) - 1) - 1) * filterTaps[i];
    }

    LOG_DEBUG(TuneFilterDecimate_i, "Setting filter taps on RF-NoC block...");

    // Send the taps to the filter RF-NoC block
    try {
        this->filter->set_taps(longFilterTaps);
    } catch(uhd::value_error &e) {
        LOG_ERROR(TuneFilterDecimate_i, "Error while setting taps on filter RF-NoC block: " << e.what());
        return false;
    } catch(...) {
        LOG_ERROR(TuneFilterDecimate_i, "Unknown error occurred while setting taps on filter RF-NoC block");
        return false;
    }

    LOG_DEBUG(TuneFilterDecimate_i, "Setting decimation factor on RF-NoC block...");

    // Set the decimation factor on the keep-one-in-N RF-NoC block
    try {
        this->decimator->set_arg("n", (int) newDecimationFactor);
    } catch(uhd::value_error &e) {
        LOG_ERROR(TuneFilterDecimate_i, "Error while setting decimation factor on decimation RF-NoC block: " << e.what());
        return false;
    } catch(...) {
        LOG_ERROR(TuneFilterDecimate_i, "Unknown error occurred while setting decimation factor on decimation RF-NoC block");
        return false;
    }

    this->taps = longFilterTaps.size();

    if (not sriChanged) {
        this->ActualOutputRate = newActualOutputRate;
        this->DecimationFactor = newDecimationFactor;
    }

    return true;
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
    this->decimatorSpp = this->decimator->get_args().cast<size_t>("spp", 1024);

    streamer_args["spp"] = boost::lexical_cast<std::string>(this->decimatorSpp);

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
    this->filterSpp = this->filter->get_args().cast<size_t>("spp", 1024);

    streamer_args["spp"] = boost::lexical_cast<std::string>(this->filterSpp);

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

void TuneFilterDecimate_i::sriChanged(const BULKIO::StreamSRI &newSRI)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    this->sri = newSRI;

    this->InputRate = 1.0 / this->sri.xdelta;

    if (not configureFD(true)) {
        LOG_ERROR(TuneFilterDecimate_i, "New SRI does not allow for configuration of filter/decimator");
    }

    this->dataShort_out->pushSRI(this->sri);

    this->receivedSRI = true;
}

void TuneFilterDecimate_i::startRxStream()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (not this->rxStreamStarted) {
        // Start continuous streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        stream_cmd.num_samps = 0;
        stream_cmd.stream_now = true;
        stream_cmd.time_spec = uhd::time_spec_t();

        this->rxStream->issue_stream_cmd(stream_cmd);

        this->rxStreamStarted = true;
    }
}

void TuneFilterDecimate_i::stopRxStream()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (this->rxStreamStarted) {
        // Start continuous streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);

        this->rxStream->issue_stream_cmd(stream_cmd);

        this->rxStreamStarted = false;
    }
}
