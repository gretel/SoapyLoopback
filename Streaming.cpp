/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe
 * Copyright (c) 2015-2017 Josh Blum

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyLoopback.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>
#include <algorithm> //min
#include <climits> //SHRT_MAX
#include <cstring> // memcpy

// Define sentinel stream pointers (C++11 compatible)
SoapySDR::Stream* const SoapyLoopback::kRxStream = reinterpret_cast<SoapySDR::Stream*>(static_cast<uintptr_t>(1));
SoapySDR::Stream* const SoapyLoopback::kTxStream = reinterpret_cast<SoapySDR::Stream*>(static_cast<uintptr_t>(2));


std::vector<std::string> SoapyLoopback::getStreamFormats(const int direction, const size_t channel) const {
    std::vector<std::string> formats;

    formats.push_back(SOAPY_SDR_CS8);
    formats.push_back(SOAPY_SDR_CS12);
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);

    return formats;
}

std::string SoapyLoopback::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {

     return SOAPY_SDR_CS12;
}

SoapySDR::ArgInfoList SoapyLoopback::getStreamArgsInfo(const int direction, const size_t channel) const {
    SoapySDR::ArgInfoList streamArgs;

    SoapySDR::ArgInfo bufflenArg;
    bufflenArg.key = "bufflen";
    bufflenArg.value = std::to_string(DEFAULT_BUFFER_LENGTH);
    bufflenArg.name = "Buffer Size";
    bufflenArg.description = "Number of bytes per buffer, multiples of 512 only.";
    bufflenArg.units = "bytes";
    bufflenArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(bufflenArg);

    SoapySDR::ArgInfo buffersArg;
    buffersArg.key = "buffers";
    buffersArg.value = std::to_string(DEFAULT_NUM_BUFFERS);
    buffersArg.name = "Ring buffers";
    buffersArg.description = "Number of buffers in the ring.";
    buffersArg.units = "buffers";
    buffersArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(buffersArg);

    SoapySDR::ArgInfo asyncbuffsArg;
    asyncbuffsArg.key = "asyncBuffs";
    asyncbuffsArg.value = "0";
    asyncbuffsArg.name = "Async buffers";
    asyncbuffsArg.description = "Number of async usb buffers (advanced).";
    asyncbuffsArg.units = "buffers";
    asyncbuffsArg.type = SoapySDR::ArgInfo::INT;

    streamArgs.push_back(asyncbuffsArg);

    return streamArgs;
}

/*******************************************************************
 * Async thread work
 ******************************************************************/

void SoapyLoopback::rx_async_operation(void)
{
    //printf("rx_async_operation\n");
    //rtlsdr_read_async(dev, &_rx_callback, this, asyncBuffs, bufferLength);
    //printf("rx_async_operation done!\n");
}

void SoapyLoopback::rx_callback(unsigned char *buf, uint32_t len)
{
    //printf("_rx_callback %d _buf_head=%d, numBuffers=%d\n", len, _buf_head, _buf_tail);

    // atomically add len to ticks but return the previous value
    unsigned long long tick = ticks.fetch_add(len);

    //overflow condition: the caller is not reading fast enough
    if (_buf_count == numBuffers)
    {
        _overflowEvent = true;
        return;
    }

    //copy into the buffer queue
    auto &buff = _buffs[_buf_tail];
    buff.tick = tick;
    buff.data.resize(len);
    std::memcpy(buff.data.data(), buf, len);

    //increment the tail pointer
    _buf_tail = (_buf_tail + 1) % numBuffers;

    //increment buffers available under lock
    //to avoid race in acquireReadBuffer wait
    {
    std::lock_guard<std::mutex> lock(_buf_mutex);
    _buf_count++;

    }

    //notify readStream()
    _buf_cond.notify_one();
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapyLoopback::setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs &args)
{

    //check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
    {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    //check the format and determine bytes per sample
    size_t bytesPerSample = 0;
    if (format == SOAPY_SDR_CF32)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
        bytesPerSample = 8;  // 2 * sizeof(float)
    }
    else if (format == SOAPY_SDR_CS16)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
        bytesPerSample = 4;  // 2 * sizeof(int16_t)
    }
    else if (format == SOAPY_SDR_CS12)
    {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS12.");
        bytesPerSample = 3;  // packed 12-bit I/Q
    }
    else if (format == SOAPY_SDR_CS8) {
        SoapySDR_log(SOAPY_SDR_INFO, "Using format CS8.");
        bytesPerSample = 2;  // 2 * sizeof(int8_t)
    }
    else
    {
        throw std::runtime_error(
                "setupStream invalid format '" + format
                        + "' -- Only CS8, CS12, CS16 and CF32 are supported by SoapyLoopback module.");
    }

    // Store format for loopback operations (first stream setup wins)
    if (_loopbackFormat.empty())
    {
        _loopbackFormat = format;
        _loopbackBytesPerSample = bytesPerSample;
        SoapySDR_logf(SOAPY_SDR_DEBUG, "Loopback format set to %s (%zu bytes/sample)", format.c_str(), bytesPerSample);
    }
    else if (_loopbackFormat != format)
    {
        SoapySDR_logf(SOAPY_SDR_WARNING, "Loopback: TX and RX using different formats (%s vs %s). Using %s for loopback.",
            _loopbackFormat.c_str(), format.c_str(), _loopbackFormat.c_str());
    }

    // Parse buffer configuration
    size_t localBufferLength = DEFAULT_BUFFER_LENGTH;
    if (args.count("bufflen") != 0)
    {
        try
        {
            int bufferLength_in = std::stoi(args.at("bufflen"));
            if (bufferLength_in > 0)
            {
                localBufferLength = bufferLength_in;
            }
        }
        catch (const std::invalid_argument &){}
    }

    size_t localNumBuffers = DEFAULT_NUM_BUFFERS;
    if (args.count("buffers") != 0)
    {
        try
        {
            int numBuffers_in = std::stoi(args.at("buffers"));
            if (numBuffers_in > 0)
            {
                localNumBuffers = numBuffers_in;
            }
        }
        catch (const std::invalid_argument &){}
    }

    SoapySDR_logf(SOAPY_SDR_DEBUG, "Loopback setupStream: direction=%s, buffer length %zu, %zu buffers",
        direction == SOAPY_SDR_RX ? "RX" : "TX", localBufferLength, localNumBuffers);

    if (direction == SOAPY_SDR_RX)
    {
        // RX stream setup: allocate RX buffers
        if (_buffs.empty())
        {
            bufferLength = localBufferLength;
            numBuffers = localNumBuffers;

            asyncBuffs = 0;
            if (args.count("asyncBuffs") != 0)
            {
                try
                {
                    int asyncBuffs_in = std::stoi(args.at("asyncBuffs"));
                    if (asyncBuffs_in > 0)
                    {
                        asyncBuffs = asyncBuffs_in;
                    }
                }
                catch (const std::invalid_argument &){}
            }

            // Initialize RX fifo
            _buf_tail = 0;
            _buf_count = 0;
            _buf_head = 0;

            // Allocate RX buffers
            _buffs.resize(numBuffers);
            for (auto &buff : _buffs) buff.data.reserve(bufferLength);
            for (auto &buff : _buffs) buff.data.resize(bufferLength);

            SoapySDR_logf(SOAPY_SDR_DEBUG, "Loopback: RX buffers allocated (%zu buffers, %zu bytes each)", numBuffers, bufferLength);
        }

        _rxStreamSetup = true;
        return kRxStream;
    }
    else
    {
        // TX stream setup: allocate loopback buffers and enable loopback
        if (_loopback_buffs.empty())
        {
            _txBufferLength = localBufferLength * 4;
            _txNumBuffers = localNumBuffers;

            // Initialize loopback ring buffer (TX -> RX path)
            _loopback_head = 0;
            _loopback_tail = 0;
            _loopback_count = 0;
            _loopback_overflow = false;

            // Allocate loopback buffers
            _loopback_buffs.resize(_txNumBuffers);
            for (auto &buff : _loopback_buffs) buff.data.reserve(_txBufferLength);
            for (auto &buff : _loopback_buffs) buff.data.resize(_txBufferLength);

            SoapySDR_logf(SOAPY_SDR_DEBUG, "Loopback: TX loopback buffers allocated (%zu buffers, %zu bytes each)", _txNumBuffers, _txBufferLength);
        }

        _loopbackEnabled = true;
        _txStreamSetup = true;
        return kTxStream;
    }
}

void SoapyLoopback::closeStream(SoapySDR::Stream *stream)
{
    this->deactivateStream(stream, 0, 0);

    if (stream == kRxStream)
    {
        _buffs.clear();
        _rxStreamSetup = false;
    }
    else if (stream == kTxStream)
    {
        _loopback_buffs.clear();
        _loopback_head = 0;
        _loopback_tail = 0;
        _loopback_count = 0;
        _loopback_overflow = false;
        _loopbackEnabled = false;
        _txStreamSetup = false;
    }
}

size_t SoapyLoopback::getStreamMTU(SoapySDR::Stream *stream) const
{
    const size_t bps = _loopbackBytesPerSample > 0 ? _loopbackBytesPerSample : BYTES_PER_SAMPLE;
    return bufferLength / bps;
}

int SoapyLoopback::activateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs,
        const size_t numElems)
{
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

    if (stream == kRxStream)
    {
        // RX activation
        resetBuffer = true;
        bufferedElems = 0;

        // Start the async thread only when loopback is NOT enabled
        // (async thread provides synthetic data for RX-only usage)
        if (not _loopbackEnabled and not _rx_async_thread.joinable())
        {
            _rx_async_thread = std::thread(&SoapyLoopback::rx_async_operation, this);
        }
    }
    else if (stream == kTxStream)
    {
        // TX activation
        _txActive = true;
        _loopbackEnabled = true;
    }

    return 0;
}

int SoapyLoopback::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

    if (stream == kRxStream)
    {
        // Deactivate RX: join the async thread if running
        if (_rx_async_thread.joinable())
        {
            _rx_async_thread.join();
        }
    }
    else if (stream == kTxStream)
    {
        // Deactivate TX: signal any waiting readers
        _txActive = false;
        _loopback_cond.notify_all();
    }

    return 0;
}

int SoapyLoopback::readStream(
        SoapySDR::Stream *stream,
        void * const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs)
{
    //drop remainder buffer on reset
    if (resetBuffer and bufferedElems != 0)
    {
        bufferedElems = 0;
        this->releaseReadBuffer(stream, _currentHandle);
    }

    //this is the user's buffer for channel 0
    void *buff0 = buffs[0];

    //are elements left in the buffer? if not, do a new read.
    if (bufferedElems == 0)
    {
        int ret = this->acquireReadBuffer(stream, _currentHandle, (const void **)&_currentBuff, flags, timeNs, timeoutUs);
        if (ret < 0) return ret;
        bufferedElems = ret;
    }

    //otherwise just update return time to the current tick count
    else
    {
        flags |= SOAPY_SDR_HAS_TIME;
        timeNs = SoapySDR::ticksToTimeNs(bufTicks, sampleRate);
    }

    size_t returnedElems = std::min(bufferedElems, numElems);

    // Determine bytes per sample based on loopback mode
    // In loopback mode use the stored format, otherwise CS8 (2 bytes) for synthetic data
    const size_t bytesPerSample = _loopbackEnabled ? _loopbackBytesPerSample : BYTES_PER_SAMPLE;

    // Copy data to user's buffer
    std::memcpy(buff0, _currentBuff, returnedElems * bytesPerSample);

    //bump variables for next call into readStream
    bufferedElems -= returnedElems;
    _currentBuff += returnedElems * bytesPerSample;
    bufTicks += returnedElems; //for the next call to readStream if there is a remainder

    //return number of elements written to buff0
    if (bufferedElems != 0) flags |= SOAPY_SDR_MORE_FRAGMENTS;
    else this->releaseReadBuffer(stream, _currentHandle);
    return returnedElems;
}

/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapyLoopback::getNumDirectAccessBuffers(SoapySDR::Stream *stream)
{
    return _buffs.size();
}

int SoapyLoopback::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs)
{
    buffs[0] = (void *)_buffs[handle].data.data();
    return 0;
}

int SoapyLoopback::acquireReadBuffer(
    SoapySDR::Stream *stream,
    size_t &handle,
    const void **buffs,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    // When loopback is enabled, read from the loopback buffer (TX -> RX)
    if (_loopbackEnabled)
    {
        // Handle loopback overflow
        if (_loopback_overflow)
        {
            _loopback_head = (_loopback_head + _loopback_count.exchange(0)) % _txNumBuffers;
            _loopback_overflow = false;
            SoapySDR::log(SOAPY_SDR_SSI, "O");
            return SOAPY_SDR_OVERFLOW;
        }

        // Wait for loopback data if none available
        if (_loopback_count == 0)
        {
            std::unique_lock<std::mutex> lock(_loopback_mutex);
            _loopback_cond.wait_for(lock, std::chrono::microseconds(timeoutUs),
                [this]{ return _loopback_count != 0; });
            if (_loopback_count == 0) return SOAPY_SDR_TIMEOUT;
        }

        // Extract from loopback buffer
        handle = _loopback_head;
        _loopback_head = (_loopback_head + 1) % _txNumBuffers;

        auto &lbuff = _loopback_buffs[handle];
        bufTicks = lbuff.tick;
        timeNs = SoapySDR::ticksToTimeNs(lbuff.tick, sampleRate);
        buffs[0] = (void *)lbuff.data.data();
        flags = SOAPY_SDR_HAS_TIME;

        // Use stored bytes per sample for the loopback format
        size_t numElems = lbuff.data.size() / _loopbackBytesPerSample;
        _loopback_count--;
        return numElems;
    }

    // Original behavior: read from synthetic RX buffer
    //reset is issued by various settings
    //to drain old data out of the queue
    if (resetBuffer)
    {
        //drain all buffers from the fifo
        _buf_head = (_buf_head + _buf_count.exchange(0)) % numBuffers;
        resetBuffer = false;
        _overflowEvent = false;
    }

    //handle overflow from the rx callback thread
    if (_overflowEvent)
    {
        //drain the old buffers from the fifo
        _buf_head = (_buf_head + _buf_count.exchange(0)) % numBuffers;
        _overflowEvent = false;
        SoapySDR::log(SOAPY_SDR_SSI, "O");
        return SOAPY_SDR_OVERFLOW;
    }

    //wait for a buffer to become available
    if (_buf_count == 0)
    {
        std::unique_lock <std::mutex> lock(_buf_mutex);
        _buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs), [this]{return _buf_count != 0;});
        if (_buf_count == 0) return SOAPY_SDR_TIMEOUT;
    }

    //extract handle and buffer
    handle = _buf_head;
    _buf_head = (_buf_head + 1) % numBuffers;
    bufTicks = _buffs[handle].tick;
    timeNs = SoapySDR::ticksToTimeNs(_buffs[handle].tick, sampleRate);
    buffs[0] = (void *)_buffs[handle].data.data();
    flags = SOAPY_SDR_HAS_TIME;

    //return number available
    return _buffs[handle].data.size() / BYTES_PER_SAMPLE;
}

void SoapyLoopback::releaseReadBuffer(
    SoapySDR::Stream *stream,
    const size_t handle)
{
    // In loopback mode, _loopback_count is already decremented in acquireReadBuffer
    // so we don't need to do anything here
    if (_loopbackEnabled)
    {
        return;
    }

    //TODO this wont handle out of order releases
    _buf_count--;
}

/*******************************************************************
 * TX Stream API - writeStream
 ******************************************************************/

int SoapyLoopback::writeStream(
        SoapySDR::Stream *stream,
        const void * const *buffs,
        const size_t numElems,
        int &flags,
        const long long timeNs,
        const long timeoutUs)
{
    if (!_txActive)
    {
        SoapySDR_log(SOAPY_SDR_ERROR, "writeStream: TX not active");
        return SOAPY_SDR_STREAM_ERROR;
    }

    if (buffs == nullptr || buffs[0] == nullptr)
    {
        SoapySDR_log(SOAPY_SDR_ERROR, "writeStream: null buffer pointer");
        return SOAPY_SDR_STREAM_ERROR;
    }

    // Use the stored bytes per sample for the loopback format
    const size_t numBytes = numElems * _loopbackBytesPerSample;

    // Get input buffer
    const char *input = (const char *)buffs[0];

    // Check for overflow condition
    if (_loopback_count >= _txNumBuffers)
    {
        _loopback_overflow = true;
        SoapySDR::log(SOAPY_SDR_SSI, "U");  // Underflow on TX side means overflow on loopback
        return SOAPY_SDR_OVERFLOW;
    }

    // Get the current tick for timestamps
    unsigned long long tick = ticks.fetch_add(numElems);

    // Copy data into loopback ring buffer
    {
        std::lock_guard<std::mutex> lock(_loopback_mutex);

        auto &buff = _loopback_buffs[_loopback_tail];
        buff.tick = tick;
        buff.data.resize(numBytes);
        std::memcpy(buff.data.data(), input, numBytes);

        // Increment tail pointer
        _loopback_tail = (_loopback_tail + 1) % _txNumBuffers;
        _loopback_count++;
    }

    // Notify any waiting readers
    _loopback_cond.notify_one();
    return numElems;
}
