/**************************************************************************

    This is the component code. This file contains the child class where
    custom functionality can be added to the component. Custom
    functionality to the base class can be extended here. Access to
    the ports can also be done from this class

**************************************************************************/

#include "RFNoC_Utils.h"
#include "TuneFilterDecimate.h"

PREPARE_LOGGING(TuneFilterDecimate_i)

TuneFilterDecimate_i::TuneFilterDecimate_i(const char *uuid, const char *label) :
    TuneFilterDecimate_base(uuid, label),
    decimatorPort(-1),
    decimatorSpp(512),
    eob(false),
    expectEob(false),
    filterPort(-1),
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

    // Release the threads if necessary
    if (this->rxThread) {
        LOG_DEBUG(TuneFilterDecimate_i, "Stopping RX thread");
        this->rxThread->stop();
        LOG_DEBUG(TuneFilterDecimate_i, "Deleting RX thread");
        delete this->rxThread;
    }

    if (this->txThread) {
        LOG_DEBUG(TuneFilterDecimate_i, "Stopping TX thread");
        this->txThread->stop();
        LOG_DEBUG(TuneFilterDecimate_i, "Deleting RX thread");
        delete this->txThread;
    }

    // Reset the RF-NoC blocks
    if (this->decimator.get()) {
        LOG_DEBUG(TuneFilterDecimate_i, "Clearing decimator");
        this->decimator->clear();
    }

    if (this->filter.get()) {
        LOG_DEBUG(TuneFilterDecimate_i, "Clearing filter");
        this->filter->clear();
    }

    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__ << " finished");
}

void TuneFilterDecimate_i::constructor()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Find available sinks and sources
    BlockInfo decimatorInfo = findAvailableChannel(this->usrp, "decimate");
    BlockInfo firInfo = findAvailableChannel(this->usrp, "FIR");

    // Without either of these, there's no need to continue
    if (not uhd::rfnoc::block_id_t::is_valid_block_id(decimatorInfo.blockID)) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to find RF-NoC block with hint 'decimate'");
        throw CF::LifeCycle::InitializeError();
    } else if (decimatorInfo.port == size_t(-1)) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to find RF-NoC block with hint 'decimate' with available port");
        throw CF::LifeCycle::InitializeError();
    }

    if (not uhd::rfnoc::block_id_t::is_valid_block_id(firInfo.blockID)) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to find RF-NoC block with hint 'FIR'");
        throw CF::LifeCycle::InitializeError();
    } else if (firInfo.port == size_t(-1)) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to find RF-NoC block with hint 'FIR' with available port");
        throw CF::LifeCycle::InitializeError();
    }

    // Get the pointers to the blocks
    this->decimator = this->usrp->get_block_ctrl<uhd::rfnoc::block_ctrl_base>(decimatorInfo.blockID);
    this->decimatorPort = decimatorInfo.port;
    this->filter = this->usrp->get_block_ctrl<uhd::rfnoc::fir_block_ctrl>(firInfo.blockID);
    this->filterPort = firInfo.port;

    if (not this->decimator.get()) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC block with ID: " << this->decimator->get_block_id());
        throw CF::LifeCycle::InitializeError();
    } else {
        LOG_DEBUG(TuneFilterDecimate_i, "Got the block: " << this->decimator->get_block_id());
    }

    if (not this->filter.get()) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC block with ID: " << this->filter->get_block_id());
        throw CF::LifeCycle::InitializeError();
    } else {
        LOG_DEBUG(TuneFilterDecimate_i, "Got the block: " << this->filter->get_block_id());
    }

    // Create a graph
    this->graph = this->usrp->create_graph("TuneFilterDecimate_" + this->_identifier);

    // Without this, there's no need to continue
    if (not this->graph.get()) {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC graph");
        throw CF::LifeCycle::InitializeError();
    }

    // Connect the blocks
    this->graph->connect(this->filter->get_block_id(), this->filterPort, this->decimator->get_block_id(), this->decimatorPort);

    // Setup based on properties initially


    // Register property change listeners
    addPropertyListener(DesiredOutputRate, this, &TuneFilterDecimate_i::DesiredOutputRateChanged); //configureFilter
    addPropertyListener(FilterBW, this, &TuneFilterDecimate_i::FilterBWChanged); //configureFilter
    addPropertyListener(filterProps, this, &TuneFilterDecimate_i::filterPropsChanged); //configureFilter
    addPropertyListener(TuningIF, this, &TuneFilterDecimate_i::TuningIFChanged); //configureTuner
    addPropertyListener(TuningNorm, this, &TuneFilterDecimate_i::TuningNormChanged); //configureTuner
    addPropertyListener(TuningRF, this, &TuneFilterDecimate_i::TuningRFChanged); //configureTuner

    // Alert the persona of the block IDs
    if (this->blockInfoChange) {
        std::vector<BlockInfo> blocks(2);

        blocks[0] = firInfo;
        blocks[1] = decimatorInfo;

        this->blockInfoChange(this->_identifier, blocks);
    }

    LOG_DEBUG(TuneFilterDecimate_i, "Adding a stream listener");

    // Add an SRI change listener
    this->dataShort_in->addStreamListener(this, &TuneFilterDecimate_i::streamChanged);

    this->dataShort_out->setNewConnectListener(this, &TuneFilterDecimate_i::newConnection);
    this->dataShort_out->setNewDisconnectListener(this, &TuneFilterDecimate_i::newDisconnection);
}

// Service functions for RX and TX
int TuneFilterDecimate_i::rxServiceFunction()
{
    // Perform RX, if necessary
    if (this->rxStream.get()) {
        // Don't bother doing anything until the SRI has been received
        if (not this->receivedSRI) {
            LOG_DEBUG(TuneFilterDecimate_i, "RX Thread active but no SRI has been received");
            return NOOP;
        }

        // Recv from the block
        uhd::rx_metadata_t md;

        size_t samplesRead = 0;
        size_t samplesToRead = this->output.size();

        while (samplesRead < this->output.size()) {
            LOG_DEBUG(TuneFilterDecimate_i, "Calling recv on the rx_stream");

            size_t num_rx_samps = this->rxStream->recv(&output.front() + samplesRead, samplesToRead, md, 1.0);

            // Check the meta data for error codes
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                LOG_ERROR(TuneFilterDecimate_i, "Timeout while streaming");
                return NOOP;
            } else if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                LOG_WARN(TuneFilterDecimate_i, "Overflow while streaming");
            } else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                LOG_WARN(TuneFilterDecimate_i, md.strerror());
                this->rxStreamStarted = false;
                startRxStream();
                return NOOP;
            }

            LOG_DEBUG(TuneFilterDecimate_i, "RX Thread Requested " << samplesToRead << " samples");
            LOG_DEBUG(TuneFilterDecimate_i, "RX Thread Received " << num_rx_samps << " samples");

            samplesRead += num_rx_samps;
            samplesToRead -= num_rx_samps;
        }

        // Get the time stamps from the meta data
        BULKIO::PrecisionUTCTime rxTime;

        rxTime.twsec = md.time_spec.get_full_secs();
        rxTime.tfsec = md.time_spec.get_frac_secs();

        // Write the data to the output stream
        if (md.end_of_burst and this->expectEob) {
            this->dataShort_out->pushPacket((short *) this->output.data(), this->output.size() * 2, rxTime, false, this->sri.streamID._ptr);
            this->expectEob = false;
        } else {
            this->dataShort_out->pushPacket((short *) this->output.data(), this->output.size() * 2, rxTime, md.end_of_burst, this->sri.streamID._ptr);
        }
    }

    return NORMAL;
}

int TuneFilterDecimate_i::txServiceFunction()
{
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

        md.end_of_burst = this->eob;
        md.has_time_spec = true;
        md.time_spec = uhd::time_spec_t(time.twsec, time.tfsec);

        if (this->eob) {
            this->eob = false;
            this->expectEob = true;
        }

        // Send the data
        size_t samplesSent = 0;
        size_t samplesToSend = blockSize;

        while (samplesToSend != 0) {
            // Send the data
            size_t num_tx_samps = this->txStream->send(block + samplesSent, samplesToSend, md, 1);

            samplesSent += num_tx_samps;
            samplesToSend -= num_tx_samps;

            LOG_DEBUG(TuneFilterDecimate_i, "TX Thread Sent " << num_tx_samps << " samples");
        }

        // On EOS, forward to the RF-NoC Block
        if (packet->EOS) {
            LOG_DEBUG(TuneFilterDecimate_i, "EOS");

            md.end_of_burst = true;

            std::vector<std::complex<short> > empty;
            this->txStream->send(&empty.front(), empty.size(), md);
        }

        delete packet;
        return NORMAL;
    }

    return NOOP;
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
 * A method which allows a callback to be set for the block info changing. This
 * callback should point back to the persona to alert it of the component's
 * block IDs and ports
 */
void TuneFilterDecimate_i::setBlockInfoCallback(blockInfoCallback cb)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    this->blockInfoChange = cb;
}

void TuneFilterDecimate_i::setNewIncomingConnectionCallback(connectionCallback cb)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    this->newIncomingConnectionCallback = cb;
}

void TuneFilterDecimate_i::setNewOutgoingConnectionCallback(connectionCallback cb)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    this->newOutgoingConnectionCallback = cb;
}

void TuneFilterDecimate_i::setRemovedIncomingConnectionCallback(connectionCallback cb)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    this->removedIncomingConnectionCallback = cb;
}

void TuneFilterDecimate_i::setRemovedOutgoingConnectionCallback(connectionCallback cb)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    this->removedOutgoingConnectionCallback = cb;
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
        if (not retrieveRxStream()) {
            LOG_WARN(TuneFilterDecimate_i, "Failed to retrieve RX stream");
            return;
        }

        // Create the receive buffer
        this->output.resize((0.8 * bulkio::Const::MAX_TRANSFER_BYTES / sizeof(std::complex<short>)));

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
        if (not retrieveTxStream()) {
            LOG_WARN(TuneFilterDecimate_i, "Failed to retrieve TX stream");
            return;
        }

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
    std::map<std::string, bool>::iterator it = this->streamMap.find(stream.streamID());

    bool newIncomingConnection = (it == this->streamMap.end());
    bool removedIncomingConnection =(it != this->streamMap.end() and stream.eos());

    if (newIncomingConnection) {
        LOG_DEBUG(TuneFilterDecimate_i, "New incoming connection");

        if (this->newIncomingConnectionCallback) {
            this->newIncomingConnectionCallback(stream.streamID(), this->dataShort_in->_this()->_hash(HASH_SIZE));
        }

        this->streamMap[stream.streamID()] = true;
    } else if (removedIncomingConnection) {
        LOG_DEBUG(TuneFilterDecimate_i, "Removed incoming connection");

        if (this->removedIncomingConnectionCallback) {
            this->removedIncomingConnectionCallback(stream.streamID(), this->dataShort_in->_this()->_hash(HASH_SIZE));
        }

        this->streamMap.erase(it);
    }

    LOG_DEBUG(TuneFilterDecimate_i, "Got SRI for stream ID: " << stream.streamID());

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

    float newDecimationFactor = round(this->InputRate / this->DesiredOutputRate);
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

    // Set the rates on the DDC RF-NoC block
    try {
        uhd::device_addr_t args;

        args["decimation_factor"] = boost::lexical_cast<std::string>(newDecimationFactor);

        this->decimator->set_args(args, this->decimatorPort);
    } catch(uhd::value_error &e) {
        LOG_ERROR(TuneFilterDecimate_i, "Error while setting decimation factor on decimation RF-NoC block: " << e.what());
        return false;
    } catch(...) {
        LOG_ERROR(TuneFilterDecimate_i, "Unknown error occurred while setting decimation factor on decimator RF-NoC block");
        return false;
    }

    // Set the property for the number of taps
    this->taps = longFilterTaps.size();

    if (not sriChanged) {
        this->ActualOutputRate = newActualOutputRate;
        this->DecimationFactor = newDecimationFactor;
    }

    // Send EOB to update the DDC decimation
    this->eob = true;

    return true;
}

void TuneFilterDecimate_i::newConnection(const char *connectionID)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (this->newOutgoingConnectionCallback) {
        BULKIO::UsesPortStatisticsProvider_ptr port = BULKIO::UsesPortStatisticsProvider::_narrow(this->getPort(this->dataShort_out->getName().c_str()));

        this->newOutgoingConnectionCallback(connectionID, port->_hash(HASH_SIZE));
    }
}

void TuneFilterDecimate_i::newDisconnection(const char *connectionID)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (this->removedOutgoingConnectionCallback) {
        BULKIO::UsesPortStatisticsProvider_ptr port = BULKIO::UsesPortStatisticsProvider::_narrow(this->getPort(this->dataShort_out->getName().c_str()));

        this->removedOutgoingConnectionCallback(connectionID, port->_hash(HASH_SIZE));
    }
}

bool TuneFilterDecimate_i::retrieveRxStream()
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

    streamer_args["block_id"] = this->decimator->get_block_id();

    // Get the spp from the block
    this->decimatorSpp = this->decimator->get_args().cast<size_t>("spp", 1024);

    streamer_args["block_port"] = boost::lexical_cast<std::string>(this->decimatorPort);
    streamer_args["spp"] = boost::lexical_cast<std::string>(this->decimatorSpp);

    stream_args.args = streamer_args;

    LOG_DEBUG(TuneFilterDecimate_i, "Using streamer arguments: " << stream_args.args.to_string());

    // Retrieve the RX stream as specified from the device 3
    try {
        this->rxStream = this->usrp->get_rx_stream(stream_args);
    } catch(uhd::runtime_error &e) {
        LOG_ERROR(TuneFilterDecimate_i, "Failed to retrieve RX stream: " << e.what());
        return false;
    } catch(...) {
        LOG_ERROR(TuneFilterDecimate_i, "Unexpected error occurred while retrieving RX stream");
        return false;
    }

    return true;
}

bool TuneFilterDecimate_i::retrieveTxStream()
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

    streamer_args["block_id"] = this->filter->get_block_id();

    // Get the spp from the block
    this->filterSpp = this->filter->get_args().cast<size_t>("spp", 1024);

    streamer_args["block_port"] = boost::lexical_cast<std::string>(this->filterPort);
    streamer_args["spp"] = boost::lexical_cast<std::string>(this->filterSpp);

    stream_args.args = streamer_args;

    LOG_DEBUG(TuneFilterDecimate_i, "Using streamer arguments: " << stream_args.args.to_string());

    // Retrieve the TX stream as specified from the device 3
    try {
        this->txStream = this->usrp->get_tx_stream(stream_args);
    } catch(uhd::runtime_error &e) {
        LOG_ERROR(TuneFilterDecimate_i, "Failed to retrieve TX stream: " << e.what());
        return false;
    } catch(...) {
        LOG_ERROR(TuneFilterDecimate_i, "Unexpected error occurred while retrieving TX stream");
        return false;
    }

    return true;
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

        // Run recv until nothing is left
        uhd::rx_metadata_t md;
        int num_post_samps = 0;

        LOG_DEBUG(TuneFilterDecimate_i, "Emptying receive queue...");

        do {
            num_post_samps = this->rxStream->recv(&this->output.front(), this->output.size(), md, 1.0);
        } while(num_post_samps and md.error_code == uhd::rx_metadata_t::ERROR_CODE_NONE);

        LOG_DEBUG(TuneFilterDecimate_i, "Emptied receive queue");
    }
}
