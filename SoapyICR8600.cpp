// SoapySDR driver module for the IC-R8600 USB I/Q output.
//
// Thin C++ wrapper around the Swift core (icr8600_core.h). The Swift layer
// handles device bring-up, control traffic, marker stripping, phase lock, and
// asynchronous bulk streaming; this file exposes that functionality through
// the SoapySDR device API.

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>

#include <chrono>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

extern "C" {
#include "icr8600_core.h"
}

#ifndef ICR8600_DEFAULT_FIRMWARE_PATH
#define ICR8600_DEFAULT_FIRMWARE_PATH "spt_seq.json"
#endif

namespace {

struct RateEntry { double hz; uint8_t code; };
const std::vector<RateEntry> kRateTable = {
    {5120000.0, ICR8600_RATE_5_12_MSPS},
    {3840000.0, ICR8600_RATE_3_84_MSPS},
    {1920000.0, ICR8600_RATE_1_92_MSPS},
    {960000.0,  ICR8600_RATE_960_KSPS},
    {480000.0,  ICR8600_RATE_480_KSPS},
    {240000.0,  ICR8600_RATE_240_KSPS},
};

uint8_t rateCodeForHz(double hz) {
    const RateEntry *best = &kRateTable.front();
    double bestDelta = std::abs(hz - best->hz);
    for (const auto &r : kRateTable) {
        double delta = std::abs(hz - r.hz);
        if (delta < bestDelta) { best = &r; bestDelta = delta; }
    }
    return best->code;
}

double nearestSupportedRate(double hz) {
    const RateEntry *best = &kRateTable.front();
    double bestDelta = std::abs(hz - best->hz);
    for (const auto &r : kRateTable) {
        double delta = std::abs(hz - r.hz);
        if (delta < bestDelta) { best = &r; bestDelta = delta; }
    }
    return best->hz;
}

struct ICR8600Stream {
    bool wantsFloat = false;
    std::vector<int16_t> scratch;
};

} // namespace

class SoapyICR8600 : public SoapySDR::Device {
public:
    explicit SoapyICR8600(const SoapySDR::Kwargs &args) {
        std::string firmwarePath = ICR8600_DEFAULT_FIRMWARE_PATH;
        auto it = args.find("firmware");
        if (it != args.end()) firmwarePath = it->second;

        icr8600_status st = icr8600_open(firmwarePath.c_str(), &_device);
        if (st != ICR8600_OK) {
            throw std::runtime_error("icr8600_open failed with status " + std::to_string(st) +
                " (firmware path: " + firmwarePath + ")");
        }
        SoapySDR_logf(SOAPY_SDR_INFO, "SoapyICR8600: device opened (firmware=%s)", firmwarePath.c_str());
    }

    ~SoapyICR8600() override {
        if (_streamActive) deactivateStream(nullptr, 0, 0);
        if (_device) icr8600_close(_device);
    }

    std::string getDriverKey() const override { return "ICR8600"; }
    std::string getHardwareKey() const override { return "Icom IC-R8600"; }

    SoapySDR::Kwargs getHardwareInfo() const override {
        SoapySDR::Kwargs info;
        info["origin"] = "SoapyIC-R8600";
        return info;
    }

    size_t getNumChannels(const int direction) const override {
        return (direction == SOAPY_SDR_RX) ? 1 : 0;
    }

    bool getFullDuplex(const int, const size_t) const override { return false; }

    std::vector<std::string> getStreamFormats(const int direction, const size_t) const override {
        if (direction != SOAPY_SDR_RX) return {};
        return {SOAPY_SDR_CS16, SOAPY_SDR_CF32};
    }

    std::string getNativeStreamFormat(const int, const size_t, double &fullScale) const override {
        fullScale = 32768.0;
        return SOAPY_SDR_CS16;
    }

    void setSampleRate(const int direction, const size_t, const double rate) override {
        if (direction != SOAPY_SDR_RX) return;
        std::lock_guard<std::mutex> lock(_mutex);
        const double newRate = nearestSupportedRate(rate);
        const bool rateChanged = (newRate != _sampleRateHz);
        _sampleRateHz = newRate;
        if (_streamActive && rateChanged) restartStreamLocked();
    }

    double getSampleRate(const int direction, const size_t) const override {
        return (direction == SOAPY_SDR_RX) ? _sampleRateHz : 0.0;
    }

    std::vector<double> listSampleRates(const int direction, const size_t) const override {
        if (direction != SOAPY_SDR_RX) return {};
        std::vector<double> out;
        for (const auto &r : kRateTable) out.push_back(r.hz);
        return out;
    }

    void setFrequency(const int direction, const size_t, const std::string &, const double frequency,
                       const SoapySDR::Kwargs &) override {
        if (direction != SOAPY_SDR_RX) return;
        std::lock_guard<std::mutex> lock(_mutex);
        _frequencyHz = frequency;
        if (_device) {
            icr8600_set_frequency(_device, static_cast<uint64_t>(_frequencyHz));
        }
    }

    double getFrequency(const int direction, const size_t, const std::string &) const override {
        return (direction == SOAPY_SDR_RX) ? _frequencyHz : 0.0;
    }

    SoapySDR::ArgInfoList getFrequencyArgsInfo(const int, const size_t) const override { return {}; }

    std::vector<std::string> listFrequencies(const int direction, const size_t) const override {
        if (direction != SOAPY_SDR_RX) return {};
        return {"RF"};
    }

    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t, const std::string &) const override {
        if (direction != SOAPY_SDR_RX) return {};
        return {SoapySDR::Range(10000.0, 3000000000.0)};
    }

    std::vector<std::string> listGains(const int direction, const size_t) const override {
        if (direction != SOAPY_SDR_RX) return {};
        return {"ATT", "RF", "PREAMP", "IPP"};
    }

    void setGain(const int direction, const size_t, const std::string &name, const double value) override {
        if (direction != SOAPY_SDR_RX) return;
        std::lock_guard<std::mutex> lock(_mutex);
        icr8600_status st = icr8600_set_gain(_device, gainElementForName(name), value);
        if (st != ICR8600_OK) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "SoapyICR8600: setGain(%s) failed (%d)", name.c_str(), st);
        }
    }

    double getGain(const int direction, const size_t, const std::string &name) const override {
        if (direction != SOAPY_SDR_RX) return 0.0;
        double value = 0.0;
        icr8600_get_gain(_device, gainElementForName(name), &value);
        return value;
    }

    SoapySDR::Range getGainRange(const int direction, const size_t, const std::string &name) const override {
        if (direction != SOAPY_SDR_RX) return SoapySDR::Range(0.0, 0.0);
        // Step size tells SoapySDR/SDR++ to snap the control to discrete
        // hardware values instead of rendering a continuous slider.
        if (name == "ATT") return SoapySDR::Range(0.0, 30.0, 10.0);   // 0/10/20/30 dB
        if (name == "RF") return SoapySDR::Range(0.0, 255.0, 1.0);     // one-byte register, integer steps
        // PREAMP / IPP are boolean toggles: step=1.0 makes them snap to 0 or 1.
        return SoapySDR::Range(0.0, 1.0, 1.0);
    }

    // Override the aggregate/no-name gain overloads. SoapySDR::Device's
    // default implementation (lib/Device.cpp) walks listGains() in order
    // and sequentially fills each element's range from one shared gain
    // budget -- i.e. it calls the real per-name setGain() on ATT, RF,
    // PREAMP, and IPP in turn. Clients that expose a "global"/"overall"
    // gain control (e.g. SDRangel's Global Gain slider, which binds
    // directly to this no-name overload) would otherwise cascade real
    // CI-V writes across all four elements any time that one control is
    // touched. Route the aggregate concept to RF only so ATT/PREAMP/IPP
    // are never touched except via their own named sliders.
    void setGain(const int direction, const size_t channel, const double value) override {
        this->setGain(direction, channel, "RF", value);
    }

    double getGain(const int direction, const size_t channel) const override {
        return this->getGain(direction, channel, "RF");
    }

    SoapySDR::Range getGainRange(const int direction, const size_t channel) const override {
        return this->getGainRange(direction, channel, "RF");
    }

    std::vector<std::string> listAntennas(const int direction, const size_t) const override {
        if (direction != SOAPY_SDR_RX) return {};
        return {"ANT1", "ANT2"};
    }

    void setAntenna(const int direction, const size_t, const std::string &name) override {
        if (direction != SOAPY_SDR_RX) return;
        std::lock_guard<std::mutex> lock(_mutex);
        uint8_t index = (name == "ANT2") ? 1 : 0;
        icr8600_status st = icr8600_set_antenna(_device, index);
        if (st != ICR8600_OK) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "SoapyICR8600: setAntenna(%s) failed (%d)", name.c_str(), st);
        }
    }

    SoapySDR::ArgInfoList getSettingInfo() const override {
        SoapySDR::ArgInfo preamp;
        preamp.key = "preamp";
        preamp.name = "Preamp";
        preamp.type = SoapySDR::ArgInfo::BOOL;
        preamp.value = "false"; // default OFF to match startup

        SoapySDR::ArgInfo ipp;
        ipp.key = "ipp";
        ipp.name = "IP+";
        ipp.type = SoapySDR::ArgInfo::BOOL;
        ipp.value = "false"; // default OFF to match startup

        // Enumerated dropdown for the attenuator, for clients that render
        // the settings API as a proper combo box (SDR++'s soapy_source does
        // not read this; it only uses listGains/getGainRange, where ATT is
        // also exposed as a stepped slider -- see getGainRange).
        SoapySDR::ArgInfo att;
        att.key = "att";
        att.name = "Attenuator";
        att.type = SoapySDR::ArgInfo::STRING;
        att.value = "0"; // default 0 dB to match startup
        att.units = "dB";
        att.options = {"0", "10", "20", "30"};
        att.optionNames = {"0 dB", "10 dB", "20 dB", "30 dB"};

        SoapySDR::ArgInfoList out;
        out.push_back(preamp);
        out.push_back(ipp);
        out.push_back(att);
        return out;
    }

    void writeSetting(const std::string &key, const std::string &value) override {
        std::lock_guard<std::mutex> lock(_mutex);
        if (key == "preamp") {
            icr8600_status st = icr8600_set_gain(_device, ICR8600_GAIN_PREAMP, (value == "true") ? 1.0 : 0.0);
            if (st != ICR8600_OK) {
                SoapySDR_logf(SOAPY_SDR_ERROR, "SoapyICR8600: writeSetting(preamp) failed (%d)", st);
            }
        }
        else if (key == "ipp") {
            icr8600_status st = icr8600_set_gain(_device, ICR8600_GAIN_IPP, (value == "true") ? 1.0 : 0.0);
            if (st != ICR8600_OK) {
                SoapySDR_logf(SOAPY_SDR_ERROR, "SoapyICR8600: writeSetting(ipp) failed (%d)", st);
            }
        }
        else if (key == "att") {
            double db = std::atof(value.c_str());
            icr8600_status st = icr8600_set_gain(_device, ICR8600_GAIN_ATT, db);
            if (st != ICR8600_OK) {
                SoapySDR_logf(SOAPY_SDR_ERROR, "SoapyICR8600: writeSetting(att) failed (%d)", st);
            }
        }
    }

    std::string readSetting(const std::string &key) const override {
        double v = 0.0;
        if (key == "preamp") {
            icr8600_get_gain(_device, ICR8600_GAIN_PREAMP, &v);
            return (v > 0.5) ? "true" : "false";
        }
        if (key == "ipp") {
            icr8600_get_gain(_device, ICR8600_GAIN_IPP, &v);
            return (v > 0.5) ? "true" : "false";
        }
        if (key == "att") {
            icr8600_get_gain(_device, ICR8600_GAIN_ATT, &v);
            return std::to_string(static_cast<long>(v));
        }
        return "";
    }

    std::string getAntenna(const int direction, const size_t) const override {
        if (direction != SOAPY_SDR_RX) return "";
        uint8_t index = 0;
        icr8600_get_antenna(_device, &index);
        return (index == 1) ? "ANT2" : "ANT1";
    }

    SoapySDR::Stream *setupStream(const int direction, const std::string &format,
                                   const std::vector<size_t> &channels = std::vector<size_t>(),
                                   const SoapySDR::Kwargs & = SoapySDR::Kwargs()) override {
        if (direction != SOAPY_SDR_RX) {
            throw std::runtime_error("SoapyICR8600: only RX is supported");
        }
        if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32) {
            throw std::runtime_error("SoapyICR8600: only CS16 or CF32 is supported");
        }
        if (!channels.empty() && (channels.size() != 1 || channels[0] != 0)) {
            throw std::runtime_error("SoapyICR8600: only channel 0 is supported");
        }
        auto *stream = new ICR8600Stream();
        stream->wantsFloat = (format == SOAPY_SDR_CF32);
        return reinterpret_cast<SoapySDR::Stream *>(stream);
    }

    void closeStream(SoapySDR::Stream *stream) override {
        delete reinterpret_cast<ICR8600Stream *>(stream);
    }

    size_t getStreamMTU(SoapySDR::Stream *) const override {
        return 65536;
    }

    int activateStream(SoapySDR::Stream *, const int flags, const long long, const size_t) override {
        if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
        std::lock_guard<std::mutex> lock(_mutex);
        return startStreamingLocked() ? 0 : SOAPY_SDR_STREAM_ERROR;
    }

    int deactivateStream(SoapySDR::Stream *, const int flags, const long long) override {
        if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
        std::lock_guard<std::mutex> lock(_mutex);
        stopStreamingLocked();
        return 0;
    }

    int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags,
                   long long &, const long timeoutUs) override {
        if (!_streamActive) return SOAPY_SDR_STREAM_ERROR;
        flags = 0;
        auto *s = reinterpret_cast<ICR8600Stream *>(stream);
        size_t pairsWritten = 0;
        int32_t dropped = 0;

        // OVERFLOW REPORTING: icr8600_read_iq signals (via outDropped) when
        // the Swift-side FIFO overflowed and silently dropped the oldest
        // samples since the previous read. SoapySDR's convention is that a
        // driver reports discontinuities by returning SOAPY_SDR_OVERFLOW
        // from readStream on the call following the drop, which apps like
        // SDRangel/GQRX surface to the operator (e.g. an "O" indicator).
        // Without this, an operator doing real RF work has no way to know
        // sample continuity was broken.
        const auto tryRead = [&](void) -> int {
            pairsWritten = 0;
            dropped = 0;

            if (!s->wantsFloat) {
                icr8600_status st = icr8600_read_iq(_device, buffs[0], numElems, &pairsWritten, &dropped);
                if (st != ICR8600_OK) return SOAPY_SDR_STREAM_ERROR;
                if (dropped != 0) flags |= SOAPY_SDR_END_ABRUPT;
                if (pairsWritten == 0) return SOAPY_SDR_TIMEOUT;
                if (dropped != 0) return SOAPY_SDR_OVERFLOW;
                return static_cast<int>(pairsWritten);
            }

            s->scratch.resize(numElems * 2);
            icr8600_status st = icr8600_read_iq(_device, s->scratch.data(), numElems, &pairsWritten, &dropped);
            if (st != ICR8600_OK) return SOAPY_SDR_STREAM_ERROR;
            if (dropped != 0) flags |= SOAPY_SDR_END_ABRUPT;

            const int16_t *src = s->scratch.data();
            float *dst = static_cast<float *>(buffs[0]);
            constexpr float kScale = 1.0f / 32768.0f;
            for (size_t i = 0; i < pairsWritten * 2; ++i) {
                dst[i] = static_cast<float>(src[i]) * kScale;
            }

            if (pairsWritten == 0) return SOAPY_SDR_TIMEOUT;
            if (dropped != 0) return SOAPY_SDR_OVERFLOW;
            return static_cast<int>(pairsWritten);
        };

        int result = tryRead();
        if (result != SOAPY_SDR_TIMEOUT || timeoutUs <= 0) {
            return result;
        }

        constexpr long kPollSleepUs = 1'000;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);
        while (_streamActive && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
            result = tryRead();
            if (result != SOAPY_SDR_TIMEOUT) {
                return result;
            }
        }

        return SOAPY_SDR_TIMEOUT;
    }

private:
    static uint8_t gainElementForName(const std::string &name) {
        if (name == "ATT") return ICR8600_GAIN_ATT;
        if (name == "RF") return ICR8600_GAIN_RF;
        if (name == "PREAMP") return ICR8600_GAIN_PREAMP;
        return ICR8600_GAIN_IPP;
    }

    bool startStreamingLocked() {
        uint8_t rateCode = rateCodeForHz(_sampleRateHz);
        icr8600_status st = icr8600_start_streaming(_device, rateCode, static_cast<uint64_t>(_frequencyHz));
        if (st != ICR8600_OK) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "SoapyICR8600: start_streaming failed (%d)", st);
            return false;
        }
        _streamActive = true;
        return true;
    }

    void stopStreamingLocked() {
        if (!_streamActive) return;
        icr8600_stop_streaming(_device);
        _streamActive = false;
    }

    void restartStreamLocked() {
        stopStreamingLocked();
        startStreamingLocked();
    }

    icr8600_device *_device = nullptr;
    std::mutex _mutex;
    bool _streamActive = false;
    double _sampleRateHz = 5120000.0;
    double _frequencyHz = 7100000.0;
};

static std::vector<SoapySDR::Kwargs> findICR8600(const SoapySDR::Kwargs &args) {
    SoapySDR::Kwargs deviceArgs;
    deviceArgs["driver"] = "icr8600";
    deviceArgs["label"] = "Icom IC-R8600 [I/Q OUT]";
    if (args.count("firmware")) deviceArgs["firmware"] = args.at("firmware");
    return {deviceArgs};
}

static SoapySDR::Device *makeICR8600(const SoapySDR::Kwargs &args) {
    return new SoapyICR8600(args);
}

static SoapySDR::Registry registerICR8600("icr8600", &findICR8600, &makeICR8600, SOAPY_SDR_ABI_VERSION);
