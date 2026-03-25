/*
 * Standalone test suite for SoapyLoopback TX->RX loopback path.
 * Raw main() + assert, no external test framework.
 *
 * Build: cmake --build build --target test_loopback
 * Run:   ctest --test-dir build --output-on-failure --timeout 30
 */

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Modules.hpp>
#include <SoapySDR/Formats.h>
#include <SoapySDR/Errors.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// --- Helpers ---

#define TEST_BEGIN(name) std::fprintf(stderr, "TEST: %s ... ", name)
#define TEST_PASS()      std::fprintf(stderr, "PASS\n")

// assert() may be compiled out under NDEBUG; use this to keep checks active
#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "FAIL: %s at %s:%d\n", #expr, __FILE__, __LINE__); \
    std::abort(); } } while(0)

static SoapySDR::Device *openDevice()
{
    SoapySDR::KwargsList results = SoapySDR::Device::enumerate("driver=loopback");
    CHECK(!results.empty());
    SoapySDR::Device *dev = SoapySDR::Device::make(results.at(0));
    CHECK(dev != nullptr);
    return dev;
}

static void closeDevice(SoapySDR::Device *dev)
{
    SoapySDR::Device::unmake(dev);
}

// --- Test cases ---

static void test_module_load()
{
    TEST_BEGIN("module_load");
    std::string err = SoapySDR::loadModule(SOAPY_LOOPBACK_MODULE_PATH);
    CHECK(err.empty());
    TEST_PASS();
}

static void test_device_enumeration()
{
    TEST_BEGIN("device_enumeration");
    SoapySDR::KwargsList results = SoapySDR::Device::enumerate("driver=loopback");
    CHECK(results.size() >= 1);
    TEST_PASS();
}

static void test_tx_stream_setup_teardown()
{
    TEST_BEGIN("tx_stream_setup_teardown");
    SoapySDR::Device *dev = openDevice();

    SoapySDR::Stream *txStream = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32);
    CHECK(txStream != nullptr);
    dev->closeStream(txStream);

    closeDevice(dev);
    TEST_PASS();
}

static void test_tx_rx_cf32_data_integrity()
{
    TEST_BEGIN("tx_rx_cf32_data_integrity");
    SoapySDR::Device *dev = openDevice();

    // TX must be set up first to enable loopback
    SoapySDR::Stream *txStream = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32);
    SoapySDR::Stream *rxStream = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    CHECK(txStream != nullptr);
    CHECK(rxStream != nullptr);

    dev->activateStream(txStream);
    dev->activateStream(rxStream);

    const size_t N = 1024;
    // Pattern: {i, N-i} as float pairs
    std::vector<float> txBuf(N * 2);
    for (size_t i = 0; i < N; i++)
    {
        txBuf[i * 2 + 0] = static_cast<float>(i);
        txBuf[i * 2 + 1] = static_cast<float>(N - i);
    }

    int flags = 0;
    long long timeNs = 0;
    const void *txBuffs[] = { txBuf.data() };
    CHECK(dev->writeStream(txStream, txBuffs, N, flags, timeNs) == static_cast<int>(N));

    // Read back
    std::vector<float> rxBuf(N * 2, 0.0f);
    void *rxBuffs[] = { rxBuf.data() };
    flags = 0;
    timeNs = 0;
    CHECK(dev->readStream(rxStream, rxBuffs, N, flags, timeNs) == static_cast<int>(N));

    // Verify byte-exact match
    CHECK(std::memcmp(txBuf.data(), rxBuf.data(), N * sizeof(float) * 2) == 0);

    dev->deactivateStream(txStream);
    dev->deactivateStream(rxStream);
    dev->closeStream(txStream);
    dev->closeStream(rxStream);
    closeDevice(dev);
    TEST_PASS();
}

static void test_tx_rx_cs16_data_integrity()
{
    TEST_BEGIN("tx_rx_cs16_data_integrity");
    SoapySDR::Device *dev = openDevice();

    SoapySDR::Stream *txStream = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16);
    SoapySDR::Stream *rxStream = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
    CHECK(txStream != nullptr);
    CHECK(rxStream != nullptr);

    dev->activateStream(txStream);
    dev->activateStream(rxStream);

    const size_t N = 1024;
    // Pattern: {i, N-i} as int16_t pairs (4 bytes/sample)
    std::vector<int16_t> txBuf(N * 2);
    for (size_t i = 0; i < N; i++)
    {
        txBuf[i * 2 + 0] = static_cast<int16_t>(i);
        txBuf[i * 2 + 1] = static_cast<int16_t>(N - i);
    }

    int flags = 0;
    long long timeNs = 0;
    const void *txBuffs[] = { txBuf.data() };
    CHECK(dev->writeStream(txStream, txBuffs, N, flags, timeNs) == static_cast<int>(N));

    // Read back
    std::vector<int16_t> rxBuf(N * 2, 0);
    void *rxBuffs[] = { rxBuf.data() };
    flags = 0;
    timeNs = 0;
    CHECK(dev->readStream(rxStream, rxBuffs, N, flags, timeNs) == static_cast<int>(N));

    // Verify byte-exact match
    CHECK(std::memcmp(txBuf.data(), rxBuf.data(), N * sizeof(int16_t) * 2) == 0);

    dev->deactivateStream(txStream);
    dev->deactivateStream(rxStream);
    dev->closeStream(txStream);
    dev->closeStream(rxStream);
    closeDevice(dev);
    TEST_PASS();
}

static void test_rx_has_time_flag()
{
    TEST_BEGIN("rx_has_time_flag");
    SoapySDR::Device *dev = openDevice();

    SoapySDR::Stream *txStream = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32);
    SoapySDR::Stream *rxStream = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    dev->activateStream(txStream);
    dev->activateStream(rxStream);

    const size_t N = 512;
    std::vector<float> txBuf(N * 2, 1.0f);

    int flags = 0;
    long long timeNs = 0;
    const void *txBuffs[] = { txBuf.data() };
    CHECK(dev->writeStream(txStream, txBuffs, N, flags, timeNs) == static_cast<int>(N));

    std::vector<float> rxBuf(N * 2, 0.0f);
    void *rxBuffs[] = { rxBuf.data() };
    flags = 0;
    timeNs = 0;
    CHECK(dev->readStream(rxStream, rxBuffs, N, flags, timeNs) == static_cast<int>(N));

    // Verify SOAPY_SDR_HAS_TIME flag is set
    CHECK(flags & SOAPY_SDR_HAS_TIME);

    dev->deactivateStream(txStream);
    dev->deactivateStream(rxStream);
    dev->closeStream(txStream);
    dev->closeStream(rxStream);
    closeDevice(dev);
    TEST_PASS();
}

static void test_overflow_on_full_ring_buffer()
{
    TEST_BEGIN("overflow_on_full_ring_buffer");
    SoapySDR::Device *dev = openDevice();

    SoapySDR::Stream *txStream = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32);
    SoapySDR::Stream *rxStream = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    dev->activateStream(txStream);
    dev->activateStream(rxStream);

    const size_t N = 1024;
    std::vector<float> txBuf(N * 2, 0.5f);
    const void *txBuffs[] = { txBuf.data() };

    // Write 20 times without reading to fill the ring buffer (> DEFAULT_NUM_BUFFERS=15)
    bool gotOverflow = false;
    for (int i = 0; i < 20; i++)
    {
        int flags = 0;
        long long timeNs = 0;
        if (dev->writeStream(txStream, txBuffs, N, flags, timeNs) == SOAPY_SDR_OVERFLOW)
        {
            gotOverflow = true;
            break;
        }
    }
    CHECK(gotOverflow);

    dev->deactivateStream(txStream);
    dev->deactivateStream(rxStream);
    dev->closeStream(txStream);
    dev->closeStream(rxStream);
    closeDevice(dev);
    TEST_PASS();
}

static void test_multiple_write_read_cycles()
{
    TEST_BEGIN("multiple_write_read_cycles");
    SoapySDR::Device *dev = openDevice();

    SoapySDR::Stream *txStream = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32);
    SoapySDR::Stream *rxStream = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    dev->activateStream(txStream);
    dev->activateStream(rxStream);

    const size_t N = 512;

    for (int cycle = 0; cycle < 5; cycle++)
    {
        // Unique pattern per cycle
        std::vector<float> txBuf(N * 2);
        for (size_t i = 0; i < N; i++)
        {
            txBuf[i * 2 + 0] = static_cast<float>(cycle * 1000 + i);
            txBuf[i * 2 + 1] = static_cast<float>(cycle * 1000 + N - i);
        }

        int flags = 0;
        long long timeNs = 0;
        const void *txBuffs[] = { txBuf.data() };
        CHECK(dev->writeStream(txStream, txBuffs, N, flags, timeNs) == static_cast<int>(N));

        std::vector<float> rxBuf(N * 2, 0.0f);
        void *rxBuffs[] = { rxBuf.data() };
        flags = 0;
        timeNs = 0;
        CHECK(dev->readStream(rxStream, rxBuffs, N, flags, timeNs) == static_cast<int>(N));

        CHECK(std::memcmp(txBuf.data(), rxBuf.data(), N * sizeof(float) * 2) == 0);
    }

    dev->deactivateStream(txStream);
    dev->deactivateStream(rxStream);
    dev->closeStream(txStream);
    dev->closeStream(rxStream);
    closeDevice(dev);
    TEST_PASS();
}

static void test_stream_mtu()
{
    TEST_BEGIN("stream_mtu");
    SoapySDR::Device *dev = openDevice();

    SoapySDR::Stream *txStream = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32);
    CHECK(dev->getStreamMTU(txStream) > 0);

    dev->closeStream(txStream);
    closeDevice(dev);
    TEST_PASS();
}

static void test_full_duplex_flag()
{
    TEST_BEGIN("full_duplex_flag");
    SoapySDR::Device *dev = openDevice();

    CHECK(dev->getFullDuplex(SOAPY_SDR_TX, 0));
    CHECK(dev->getFullDuplex(SOAPY_SDR_RX, 0));

    closeDevice(dev);
    TEST_PASS();
}

static void test_module_unload()
{
    TEST_BEGIN("module_unload");
    std::string err = SoapySDR::unloadModule(SOAPY_LOOPBACK_MODULE_PATH);
    CHECK(err.empty());
    TEST_PASS();
}

// --- Main ---

int main()
{
    std::fprintf(stderr, "=== SoapyLoopback Test Suite ===\n");

    test_module_load();
    test_device_enumeration();
    test_tx_stream_setup_teardown();
    test_tx_rx_cf32_data_integrity();
    test_tx_rx_cs16_data_integrity();
    test_rx_has_time_flag();
    test_overflow_on_full_ring_buffer();
    test_multiple_write_read_cycles();
    test_stream_mtu();
    test_full_duplex_flag();
    test_module_unload();

    std::fprintf(stderr, "=== All tests passed ===\n");
    return 0;
}
