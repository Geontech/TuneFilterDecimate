// Component Header
#include "TuneFilterDecimate.h"

// Boost Include(s)
#include <boost/shared_ptr.hpp>

PREPARE_LOGGING(TuneFilterDecimate_i)

/*
 * Constructor(s) and/or Destructor
 */
TuneFilterDecimate_i::TuneFilterDecimate_i(const char *uuid, const char *label) :
    	TuneFilterDecimate_base(uuid, label),
		decimatorPort(-1),
		decimatorSpp(512),
		eob(false),
		expectEob(false),
		filterPort(-1),
		filterSpp(512),
		receivedSRI(false),
		rxStreamStarted(false)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);
}

TuneFilterDecimate_i::~TuneFilterDecimate_i()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Stop streaming
    stopRxStream();

    // Release the threads if necessary
    if (this->rxThread)
    {
        this->rxThread->stop();
    }

    if (this->txThread)
    {
        this->txThread->stop();
    }
}

/*
 * Public Method(s)
 */

void TuneFilterDecimate_i::releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // This function clears the component running condition so main shuts down everything
    try
    {
        stop();
    }
    catch (CF::Resource::StopError& ex)
    {
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

void TuneFilterDecimate_i::start() throw (CF::Resource::StartError, CORBA::SystemException)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    TuneFilterDecimate_base::start();

    if (this->rxThread)
    {
        startRxStream();

        this->rxThread->start();
    }

    if (this->txThread)
    {
        this->txThread->start();
    }
}

void TuneFilterDecimate_i::stop() throw (CF::Resource::StopError, CORBA::SystemException)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (this->rxThread)
    {
        if (not this->rxThread->stop())
        {
            LOG_WARN(TuneFilterDecimate_i, "RX Thread had to be killed");
        }

        stopRxStream();
    }

    if (this->txThread)
    {
        if (not this->txThread->stop())
        {
            LOG_WARN(TuneFilterDecimate_i, "TX Thread had to be killed");
        }
    }

    TuneFilterDecimate_base::stop();
}

/*
 * Public RFNoC_Component Method(s)
 */

/*
 * A method which allows the persona to set this component as an RX streamer.
 * This means the component should retrieve the data from block and then send
 * it out as bulkio data.
 */
void TuneFilterDecimate_i::setRxStreamer(uhd::rx_streamer::sptr rxStreamer)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (rxStreamer)
    {
        // This shouldn't happen
        if (this->rxStreamer)
        {
            LOG_DEBUG(TuneFilterDecimate_i, "Replacing existing RX streamer");
            return;
        }

        LOG_DEBUG(TuneFilterDecimate_i, "Attempting to set RX streamer");

        // Set the RX stream
        this->rxStreamer = rxStreamer;

        // Create the RX receive thread
        this->rxThread = boost::make_shared<RFNoC_RH::GenericThreadedComponent>(boost::bind(&TuneFilterDecimate_i::rxServiceFunction, this));

        // If the component is already started, then start the RX receive thread
        if (this->_started)
        {
            startRxStream();

            this->rxThread->start();
        }
    }
    else
    {
        // Don't clean up the stream if it's not already running
        if (not this->rxStreamer)
        {
            LOG_DEBUG(TuneFilterDecimate_i, "Attempted to unset RX streamer, but not streaming");
            return;
        }

        // Stop and delete the RX stream thread
        if (not this->rxThread->stop())
        {
            LOG_WARN(TuneFilterDecimate_i, "RX Thread had to be killed");
        }

        // Stop continuous streaming
        stopRxStream();

        // Release the RX stream pointer
        LOG_DEBUG(TuneFilterDecimate_i, "Resetting RX streamer");
        this->rxStreamer.reset();

        this->rxThread.reset();
    }
}

/*
 * A method which allows the persona to set this component as a TX streamer.
 * This means the component should retrieve the data from the bulkio port and
 *  then send it to the block.
 */
void TuneFilterDecimate_i::setTxStreamer(uhd::tx_streamer::sptr txStreamer)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (txStreamer)
    {
        // This shouldn't happen
        if (this->txStreamer)
        {
            LOG_DEBUG(TuneFilterDecimate_i, "Replacing TX streamer");
            return;
        }

        LOG_DEBUG(TuneFilterDecimate_i, "Attempting to set TX streamer");

        // Set the TX stream
        this->txStreamer = txStreamer;

        // Create the TX transmit thread
        this->txThread = boost::make_shared<RFNoC_RH::GenericThreadedComponent>(boost::bind(&TuneFilterDecimate_i::txServiceFunction, this));

        // If the component is already started, then start the TX transmit thread
        if (this->_started)
        {
            this->txThread->start();
        }
    }
    else
    {
        // Don't clean up the stream if it's not already running
        if (not this->txStreamer)
        {
            LOG_DEBUG(TuneFilterDecimate_i, "Attempted to unset TX streamer, but not streaming");
            return;
        }

        // Stop and delete the TX stream thread
        if (not this->txThread->stop())
        {
            LOG_WARN(TuneFilterDecimate_i, "TX Thread had to be killed");
        }

        // Release the TX stream pointer
        this->txStreamer.reset();

        this->txThread.reset();
    }
}

/*
 * Protected Method(s)
 */

void TuneFilterDecimate_i::constructor()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    // Construct Block Descriptors
    RFNoC_RH::BlockDescriptor decimatorBlockDescriptor, firBlockDescriptor;

    decimatorBlockDescriptor.blockId = "decimate";
    firBlockDescriptor.blockId = "FIR";

    // Grab the block pointers
    this->decimator = this->persona->getBlock(decimatorBlockDescriptor);;
    this->filter = boost::dynamic_pointer_cast<uhd::rfnoc::fir_block_ctrl>(this->persona->getBlock(firBlockDescriptor));

    if (not this->decimator)
    {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC block with ID: decimate");
        throw CF::LifeCycle::InitializeError();
    }
    else
    {
        LOG_DEBUG(TuneFilterDecimate_i, "Got the block: decimate");
    }

    if (not this->filter)
    {
        LOG_FATAL(TuneFilterDecimate_i, "Unable to retrieve RF-NoC block with ID: filter");
        throw CF::LifeCycle::InitializeError();
    }
    else
    {
        LOG_DEBUG(TuneFilterDecimate_i, "Got the block: filter");
    }

    this->decimatorPort = decimatorBlockDescriptor.port;
    this->filterPort = firBlockDescriptor.port;

    // Connect the blocks
    if (not this->persona->connectBlocks(firBlockDescriptor, decimatorBlockDescriptor))
	{
    	LOG_FATAL(TuneFilterDecimate_i, "Failed to connect filter to decimator");
    	throw CF::LifeCycle::InitializeError();
	}

    // Setup based on properties initially

    // Alert the persona of stream descriptors for this component
    RFNoC_RH::StreamDescriptor rxStreamDescriptor, txStreamDescriptor;

    rxStreamDescriptor.cpuFormat = "sc16";
    rxStreamDescriptor.otwFormat = "sc16";
    rxStreamDescriptor.streamArgs["block_id"] = this->decimator->get_block_id();
    rxStreamDescriptor.streamArgs["block_port"] = this->decimatorPort;

    txStreamDescriptor.cpuFormat = "sc16";
	txStreamDescriptor.otwFormat = "sc16";
	txStreamDescriptor.streamArgs["block_id"] = this->filter->get_block_id();
	txStreamDescriptor.streamArgs["block_port"] = this->filterPort;

    // Get the spps from the blocks
    this->decimatorSpp = this->decimator->get_args().cast<size_t>("spp", 512);
    this->filterSpp = this->filter->get_args().cast<size_t>("spp", 512);

    rxStreamDescriptor.streamArgs["spp"] = boost::lexical_cast<std::string>(this->decimatorSpp);
    txStreamDescriptor.streamArgs["spp"] = boost::lexical_cast<std::string>(this->filterSpp);

    this->persona->setRxStreamDescriptor(this->identifier(), rxStreamDescriptor);
    this->persona->setTxStreamDescriptor(this->identifier(), txStreamDescriptor);

    // Register property change listeners
    addPropertyListener(DesiredOutputRate, this, &TuneFilterDecimate_i::DesiredOutputRateChanged); //configureFilter
    addPropertyListener(FilterBW, this, &TuneFilterDecimate_i::FilterBWChanged); //configureFilter
    addPropertyListener(filterProps, this, &TuneFilterDecimate_i::filterPropsChanged); //configureFilter
    addPropertyListener(TuningIF, this, &TuneFilterDecimate_i::TuningIFChanged); //configureTuner
    addPropertyListener(TuningNorm, this, &TuneFilterDecimate_i::TuningNormChanged); //configureTuner
    addPropertyListener(TuningRF, this, &TuneFilterDecimate_i::TuningRFChanged); //configureTuner

    // Add an SRI change listener
    this->dataShort_in->addStreamListener(this, &TuneFilterDecimate_i::streamChanged);

    // Add a stream listener
    this->dataShort_out->setNewConnectListener(this, &TuneFilterDecimate_i::newConnection);
    this->dataShort_out->setNewDisconnectListener(this, &TuneFilterDecimate_i::newDisconnection);

    // Create the receive buffer
    this->output.resize((0.8 * bulkio::Const::MAX_TRANSFER_BYTES / sizeof(std::complex<short>)));
}

// Service functions for RX and TX
int TuneFilterDecimate_i::rxServiceFunction()
{
    // Perform RX, if necessary
    if (this->rxStreamer)
    {
        // Don't bother doing anything until the SRI has been received
        if (not this->receivedSRI)
        {
            LOG_TRACE(TuneFilterDecimate_i, "RX Thread active but no SRI has been received");
            return NOOP;
        }

        // Recv from the block
        uhd::rx_metadata_t md;

        size_t samplesRead = 0;
        size_t samplesToRead = this->output.size();

        while (samplesRead < this->output.size())
        {
            LOG_DEBUG(TuneFilterDecimate_i, "Calling recv on the rx_stream");

            size_t num_rx_samps = this->rxStreamer->recv(&output.front() + samplesRead, samplesToRead, md, 1.0);

            // Check the meta data for error codes
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
            {
                LOG_ERROR(TuneFilterDecimate_i, "Timeout while streaming");
                return NOOP;
            }
            else if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
            {
                LOG_WARN(TuneFilterDecimate_i, "Overflow while streaming");
            }
            else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
            {
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
        if (md.end_of_burst and this->expectEob)
        {
            this->dataShort_out->pushPacket((short *) this->output.data(), this->output.size() * 2, rxTime, false, this->sri.streamID._ptr);
            this->expectEob = false;
        }
        else
        {
            this->dataShort_out->pushPacket((short *) this->output.data(), this->output.size() * 2, rxTime, md.end_of_burst, this->sri.streamID._ptr);
        }
    }

    return NORMAL;
}

int TuneFilterDecimate_i::txServiceFunction()
{
    // Perform TX, if necessary
    if (this->txStreamer)
    {
        // Wait on input data
        bulkio::InShortPort::DataTransferType *packet = this->dataShort_in->getPacket(bulkio::Const::BLOCKING);

        if (not packet)
        {
            return NOOP;
        }

        // Respond to the SRI changing
        if (packet->sriChanged)
        {
            sriChanged(packet->SRI);
        }

        // Prepare the metadata
        uhd::tx_metadata_t md;
        std::complex<short> *block = (std::complex<short> *) packet->dataBuffer.data();
        size_t blockSize = packet->dataBuffer.size() / 2;

        LOG_TRACE(TuneFilterDecimate_i, "TX Thread Received " << blockSize << " samples");

        if (blockSize == 0)
        {
            LOG_TRACE(TuneFilterDecimate_i, "Skipping empty packet");
            delete packet;
            return NOOP;
        }

        // Get the timestamp to send to the RF-NoC block
        BULKIO::PrecisionUTCTime time = packet->T;

        md.end_of_burst = this->eob;
        md.has_time_spec = true;
        md.time_spec = uhd::time_spec_t(time.twsec, time.tfsec);

        if (this->eob)
        {
            this->eob = false;
            this->expectEob = true;
        }

        // Send the data
        size_t samplesSent = 0;
        size_t samplesToSend = blockSize;

        while (samplesToSend != 0)
        {
            // Send the data
            size_t num_tx_samps = this->txStreamer->send(block + samplesSent, samplesToSend, md, 1);

            samplesSent += num_tx_samps;
            samplesToSend -= num_tx_samps;

            LOG_DEBUG(TuneFilterDecimate_i, "TX Thread Sent " << num_tx_samps << " samples");
        }

        // On EOS, forward to the RF-NoC Block
        if (packet->EOS)
        {
            LOG_DEBUG(TuneFilterDecimate_i, "EOS");

            // Propagate the EOS to the RF-NoC Block
            md.end_of_burst = true;

            std::vector<std::complex<short> > empty;
            this->txStreamer->send(&empty.front(), empty.size(), md);
        }

        delete packet;
        return NORMAL;
    }

    return NOOP;
}

/*
 * Private Method(s)
 */

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
    if (sriChanged)
    {
        this->ActualOutputRate = newActualOutputRate;
        this->DecimationFactor = newDecimationFactor;
    }

    LOG_DEBUG(TuneFilterDecimate_i, "New Decimation Factor: " << newDecimationFactor);

    this->sri.xdelta = 1.0 / newActualOutputRate;

    if (newActualOutputRate < this->FilterBW)
    {
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

    for (size_t i = 0; i < availableFilterLength; ++i)
    {
        longFilterTaps[i] = (pow(2, (sizeof(short int) * 8) - 1) - 1) * filterTaps[i];
    }

    LOG_DEBUG(TuneFilterDecimate_i, "Setting filter taps on RF-NoC block...");

    // Send the taps to the filter RF-NoC block
    try
    {
        this->filter->set_taps(longFilterTaps);
    }
    catch(uhd::value_error &e)
    {
        LOG_ERROR(TuneFilterDecimate_i, "Error while setting taps on filter RF-NoC block: " << e.what());
        return false;
    }
    catch(...)
    {
        LOG_ERROR(TuneFilterDecimate_i, "Unknown error occurred while setting taps on filter RF-NoC block");
        return false;
    }

    LOG_DEBUG(TuneFilterDecimate_i, "Setting decimation factor on RF-NoC block...");

    // Set the rates on the decimator RF-NoC block
    try
    {
        uhd::device_addr_t args;

        args["decimation_factor"] = boost::lexical_cast<std::string>(newDecimationFactor);

        this->decimator->set_args(args, this->decimatorPort);
    }
    catch(uhd::value_error &e)
    {
        LOG_ERROR(TuneFilterDecimate_i, "Error while setting decimation factor on decimation RF-NoC block: " << e.what());
        return false;
    }
    catch(...)
    {
        LOG_ERROR(TuneFilterDecimate_i, "Unknown error occurred while setting decimation factor on decimator RF-NoC block");
        return false;
    }

    // Set the property for the number of taps
    this->taps = longFilterTaps.size();

    if (not sriChanged)
    {
        this->ActualOutputRate = newActualOutputRate;
        this->DecimationFactor = newDecimationFactor;
    }

    // Send EOB to update the DDC decimation
    this->eob = true;

    return true;
}

void TuneFilterDecimate_i::DesiredOutputRateChanged(const float &oldValue, const float &newValue)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (this->receivedSRI)
    {
        if (not configureFD())
        {
            LOG_ERROR(TuneFilterDecimate_i, "Unable to configure filter/decimator with requested DesiredOutputRate");
            this->DesiredOutputRate = oldValue;
        }
        else
        {
            this->dataShort_out->pushSRI(this->sri);
        }
    }
    else
    {
        LOG_WARN(TuneFilterDecimate_i, "Not designing filter until receiving SRI");
    }
}

void TuneFilterDecimate_i::FilterBWChanged(const float &oldValue, const float &newValue)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (newValue < 0.0)
    {
        LOG_WARN(TuneFilterDecimate_i, "Attempted to set FilterBW to a value less than zero");
        this->FilterBW = oldValue;
        return;
    }

    if (this->receivedSRI)
    {
        if (not configureFD())
        {
            LOG_ERROR(TuneFilterDecimate_i, "Unable to configure filter/decimator with requested FilterBW");
            this->FilterBW = oldValue;
        }
    }
    else
    {
        LOG_WARN(TuneFilterDecimate_i, "Not designing filter until receiving SRI");
    }
}

void TuneFilterDecimate_i::filterPropsChanged(const filterProps_struct &oldValue, const filterProps_struct &newValue)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (newValue.Ripple < 0.0)
    {
        LOG_WARN(TuneFilterDecimate_i, "Attempted to set filterProps.Ripple to a value less than zero");
        this->filterProps = oldValue;
        return;
    }

    if (newValue.TransitionWidth < 0.0)
    {
        LOG_WARN(TuneFilterDecimate_i, "Attempted to set filterProps.TransitionWidth to a value less than zero");
        this->filterProps = oldValue;
        return;
    }

    if (this->receivedSRI)
    {
        if (not configureFD())
        {
            LOG_ERROR(TuneFilterDecimate_i, "Unable to configure filter/decimator with requested filterProps");
            this->filterProps = oldValue;
        }
    }
    else
    {
        LOG_WARN(TuneFilterDecimate_i, "Not designing filter until receiving SRI");
    }
}

void TuneFilterDecimate_i::newConnection(const char *connectionID)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

	BULKIO::UsesPortStatisticsProvider_ptr port = BULKIO::UsesPortStatisticsProvider::_narrow(this->getPort(this->dataShort_out->getName().c_str()));

	this->persona->outgoingConnectionAdded(this->identifier(), connectionID, port->_hash(RFNoC_RH::HASH_SIZE));
}

void TuneFilterDecimate_i::newDisconnection(const char *connectionID)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

	BULKIO::UsesPortStatisticsProvider_ptr port = BULKIO::UsesPortStatisticsProvider::_narrow(this->getPort(this->dataShort_out->getName().c_str()));

	this->persona->outgoingConnectionRemoved(this->identifier(), connectionID, port->_hash(RFNoC_RH::HASH_SIZE));
}

void TuneFilterDecimate_i::sriChanged(const BULKIO::StreamSRI &newSRI)
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    this->sri = newSRI;

    this->InputRate = 1.0 / this->sri.xdelta;

    if (not configureFD(true))
    {
        LOG_ERROR(TuneFilterDecimate_i, "New SRI does not allow for configuration of filter/decimator");
    }

    this->dataShort_out->pushSRI(this->sri);

    this->receivedSRI = true;
}

void TuneFilterDecimate_i::startRxStream()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (not this->rxStreamStarted and this->rxStreamer)
    {
        // Start continuous streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        stream_cmd.num_samps = 0;
        stream_cmd.stream_now = true;
        stream_cmd.time_spec = uhd::time_spec_t();

        this->rxStreamer->issue_stream_cmd(stream_cmd);

        this->rxStreamStarted = true;
    }
}

void TuneFilterDecimate_i::stopRxStream()
{
    LOG_TRACE(TuneFilterDecimate_i, __PRETTY_FUNCTION__);

    if (this->rxStreamStarted and this->rxStreamer)
    {
        // Start continuous streaming
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);

        this->rxStreamer->issue_stream_cmd(stream_cmd);

        this->rxStreamStarted = false;

        // Run recv until nothing is left
        uhd::rx_metadata_t md;
        int num_post_samps = 0;

        LOG_DEBUG(TuneFilterDecimate_i, "Emptying receive queue...");

        do
        {
            num_post_samps = this->rxStreamer->recv(&this->output.front(), this->output.size(), md, 1.0);
        } while(num_post_samps and md.error_code == uhd::rx_metadata_t::ERROR_CODE_NONE);

        LOG_DEBUG(TuneFilterDecimate_i, "Emptied receive queue");
    }
}

void TuneFilterDecimate_i::streamChanged(bulkio::InShortPort::StreamType stream)
{
    std::map<std::string, bool>::iterator it = this->streamMap.find(stream.streamID());

    bool newIncomingConnection = (it == this->streamMap.end());
    bool removedIncomingConnection = (it != this->streamMap.end() and stream.eos());

    if (newIncomingConnection)
    {
        LOG_DEBUG(TuneFilterDecimate_i, "New incoming connection");

        this->persona->incomingConnectionAdded(this->identifier(),
        							  	  	   stream.streamID(),
											   this->dataShort_in->_this()->_hash(RFNoC_RH::HASH_SIZE));

        this->streamMap[stream.streamID()] = true;
    }
    else if (removedIncomingConnection)
    {
        LOG_DEBUG(TuneFilterDecimate_i, "Removed incoming connection");

        this->persona->incomingConnectionRemoved(this->identifier(),
        										 stream.streamID(),
												 this->dataShort_in->_this()->_hash(RFNoC_RH::HASH_SIZE));

        this->streamMap.erase(it);
    }
    else
    {
    	LOG_DEBUG(TuneFilterDecimate_i, "Existing connection changed");
    }

    LOG_DEBUG(TuneFilterDecimate_i, "Got SRI for stream ID: " << stream.streamID());

    sriChanged(stream.sri());
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
