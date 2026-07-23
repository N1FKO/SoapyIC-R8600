import Foundation
import IOKit
import IOUSBHost
import CICR8600Core

private let VID = 0x0C26
private let PID_LOADER = 0x0022
private let PID_STREAM = 0x0023
private let NBUF = 16
private let BUFSIZE = 262_144

// Cap on buffered aligned I/Q bytes if a consumer isn't draining fast enough
// (e.g. between icr8600_read_iq calls). Prevents unbounded growth; a real
// SoapySDR consumer should drain continuously so this should rarely trigger.
private let MAX_FIFO_BYTES = 16 * 1024 * 1024

// Fixed-capacity byte ring used for aligned I/Q output. When the buffer
// would overflow, the oldest bytes are discarded to make room for newer
// samples.
final class ByteRingBuffer {
    private var storage: UnsafeMutableRawPointer
    private let capacity: Int
    private var head = 0   // next byte to read
    private var count = 0  // bytes currently buffered

    init(capacity: Int) {
        self.capacity = capacity
        self.storage = UnsafeMutableRawPointer.allocate(byteCount: capacity, alignment: 1)
    }

    deinit {
        storage.deallocate()
    }

    var occupied: Int { count }
    var isEmpty: Bool { count == 0 }

    // Appends `bytes` to the tail. If the ring is full and would overflow,
    // the oldest bytes are dropped first (advancing `head`) to make room,
    // and this method returns `true` to indicate a drop occurred -- the
    // ring-buffer equivalent of the old MAX_FIFO_BYTES trim.
    @discardableResult
    func write(_ bytes: UnsafeRawBufferPointer) -> Bool {
        var dropped = false
        var n = bytes.count
        guard n > 0 else { return false }

        // If the incoming chunk alone exceeds capacity (shouldn't happen in
        // practice given BUFSIZE << MAX_FIFO_BYTES, but guard anyway), only
        // keep the tail end of it.
        if n > capacity {
            let skip = n - capacity
            n = capacity
            head = 0
            count = 0
            dropped = true
            writeInternal(bytes.baseAddress! + skip, n)
            return dropped
        }

        let freeSpace = capacity - count
        if n > freeSpace {
            let overflow = n - freeSpace
            head = (head + overflow) % capacity
            count -= overflow
            dropped = true
        }

        writeInternal(bytes.baseAddress!, n)
        return dropped
    }

    private func writeInternal(_ src: UnsafeRawPointer, _ n: Int) {
        let tail = (head + count) % capacity
        let firstLen = min(n, capacity - tail)
        memcpy(storage + tail, src, firstLen)
        if firstLen < n {
            memcpy(storage, src + firstLen, n - firstLen)
        }
        count += n
    }

    // Copies up to `maxBytes` from the head into `dst`. Returns the number
    // of bytes actually copied (may be less than maxBytes, including 0).
    func read(into dst: UnsafeMutableRawPointer, maxBytes: Int) -> Int {
        let n = min(maxBytes, count)
        guard n > 0 else { return 0 }

        let firstLen = min(n, capacity - head)
        memcpy(dst, storage + head, firstLen)
        if firstLen < n {
            memcpy(dst + firstLen, storage, n - firstLen)
        }
        head = (head + n) % capacity
        count -= n
        return n
    }
}

public final class ICR8600DeviceBox {
    let ioService: io_service_t
    let hostDevice: IOUSBHostDevice
    var interfaceService: io_service_t = 0
    var interfaceObject: IOUSBHostInterface?
    var outPipe: IOUSBHostPipe?
    var ackPipe: IOUSBHostPipe?
    var iqPipe: IOUSBHostPipe?
    var reader: AsyncReader?
    var isStreaming = false

    // Cached gain/antenna/frequency state. Defaults preserve the previous
    // startup behavior until a client calls setGain/setAntenna/setFrequency.
    var attDb: Double = 0
    var rfGain: Double = 255
    var preampOn: Double = 0
    var ippOn: Double = 0
    var antennaIndex: UInt8 = 0
    var frequencyHz: UInt64 = 7_100_000

    init(ioService: io_service_t, hostDevice: IOUSBHostDevice) {
        self.ioService = ioService
        self.hostDevice = hostDevice
    }

    deinit {
        if interfaceService != 0 { IOObjectRelease(interfaceService) }
        if ioService != 0 { IOObjectRelease(ioService) }
    }
}

// Reads raw bulk-IN data from endpoint 0x86 and performs Phase 4 alignment:
// strips the 00 80 00 80 sync markers, continuously re-locks the I/Q phase
// at every marker, and only ever hands off complete (I, Q) pairs downstream
// Aligned I/Q samples are produced by stripping sync markers and carrying
// a trailing partial sample across DMA buffer boundaries. All shared stream
// state is confined to `queue`, and each DMA buffer has a same-index scratch
// copy so alignment work can proceed independently of DMA buffer reuse.
final class AsyncReader {
    let pipe: IOUSBHostPipe
    let queue: DispatchQueue
    var buffers: [NSMutableData]
    // One scratch copy buffer per DMA buffer, same index, same fixed size.
    // This lets alignment work proceed independently of DMA buffer reuse.
    private let copyBuffers: [UnsafeMutableRawPointer]
    var totalBytes = 0
    var syncCount = 0
    var completions = 0
    var stopping = false

    // Alignment state, all confined to `queue`.
    private var expectingI = true
    private var pendingI: (UInt8, UInt8)?
    private var carryByte: UInt8?

    // Aligned, interleaved int16 LE I/Q output, ready for a consumer to
    // drain. Fixed-capacity ring buffer -- see ByteRingBuffer above for why
    // this replaced a Data-based FIFO.
    private let sampleFIFO = ByteRingBuffer(capacity: MAX_FIFO_BYTES)

    // Set whenever the FIFO overflows and drops the oldest bytes to make
    // room. Consumed and cleared by the next drain(into:maxBytes:) call so a
    // SoapySDR-style caller can surface exactly one overflow indication per
    // drop, matching readStream()'s per-call flags contract.
    private var droppedSinceLastDrain = false

    init(pipe: IOUSBHostPipe, queue: DispatchQueue, nbuf: Int, size: Int) {
        self.pipe = pipe
        self.queue = queue
        self.buffers = (0..<nbuf).map { _ in NSMutableData(length: size)! }
        self.copyBuffers = (0..<nbuf).map { _ in UnsafeMutableRawPointer.allocate(byteCount: size, alignment: 1) }
    }

    deinit {
        for b in copyBuffers { b.deallocate() }
    }

    func start() {
        queue.async {
            for (slot, b) in self.buffers.enumerated() { self.enqueue(b, slot: slot) }
        }
    }

    // `slot` is this buffer's fixed index into both `buffers` and
    // `copyBuffers` -- threaded through every re-enqueue so the completion
    // handler never has to look it up.
    func enqueue(_ data: NSMutableData, slot: Int) {
        data.length = BUFSIZE
        do {
            try pipe.enqueueIORequest(with: data, completionTimeout: 3.0) { [weak self] status, bytes in
                guard let self = self else { return }

                // Copy the just-completed bytes into this slot's dedicated
                // scratch buffer before re-arming `data` below.
                let copy = self.copyBuffers[slot]
                if bytes > 0 {
                    memcpy(copy, data.bytes, bytes)
                }

                // Re-enqueue immediately regardless of alignment-processing
                // state so the USB transfer pipeline never stalls waiting
                // on `queue`. Safe now: the bytes we still need are already
                // copied out above, so overwriting `data` here can't lose
                // or corrupt anything `process` hasn't read yet.
                if !self.stopping { self.enqueue(data, slot: slot) }

                // Do the actual alignment/FIFO work confined to `queue`,
                // asynchronously, against the private copy -- so this
                // completion handler (which may be invoked from an IOKit
                // context that is NOT guaranteed to be `queue`) never
                // touches shared state directly and never blocks on it.
                if bytes > 0 {
                    self.queue.async {
                        self.process(copy, bytes)
                    }
                }
            }
        } catch {
            print("enqueue failed: \(error.localizedDescription)")
        }
    }

    // Always runs on `queue`, dispatched explicitly via `queue.async` from
    // the completion handler above -- never called directly from a
    // completion context that might run on a different queue. This is the
    // single serialization point for all alignment state and `sampleFIFO`
    // mutation, matching `drain()`'s `queue.sync` access below.
    func process(_ buffer: UnsafeRawPointer, _ n: Int) {
        completions += 1
        totalBytes += n
        let p = buffer.assumingMemoryBound(to: UInt8.self)

        var aligned = [UInt8]()
        aligned.reserveCapacity(n)

        var i = 0
        // Fold in a leftover byte from the previous buffer, if any, so every
        // 2-byte sample we examine is buffer-boundary-safe.
        var byte0: UInt8? = carryByte
        carryByte = nil

        func nextSampleBytes() -> (UInt8, UInt8)? {
            if let b0 = byte0 {
                byte0 = nil
                guard i < n else { carryByte = b0; return nil }
                let b1 = p[i]; i += 1
                return (b0, b1)
            }
            guard i + 1 < n else {
                if i < n { carryByte = p[i] }
                return nil
            }
            let b0 = p[i], b1 = p[i + 1]
            i += 2
            return (b0, b1)
        }

        while true {
            guard let (lo, hi) = nextSampleBytes() else { break }
            let raw = UInt16(lo) | (UInt16(hi) << 8)

            if raw == 0x8000 {
                // Markers are two consecutive 0x8000 samples. Peek the next
                // sample; if it's also 0x8000, this is a real marker.
                guard let (lo2, hi2) = nextSampleBytes() else {
                    // Buffer ended mid-marker candidate; treat as boundary
                    // carry via carryByte/pendingI state already set above.
                    break
                }
                let raw2 = UInt16(lo2) | (UInt16(hi2) << 8)
                if raw2 == 0x8000 {
                    syncCount += 1
                    // Re-lock: the marker is always immediately followed by I.
                    // Drop any unpaired pendingI — it belongs to a broken pair.
                    pendingI = nil
                    expectingI = true
                    continue
                } else {
                    // Lone 0x8000 without a matching partner: 0x8000 never
                    // occurs in real data, so drop this one anomalous sample
                    // and process the other normally below by re-injecting it.
                    if expectingI {
                        pendingI = (lo2, hi2)
                        expectingI = false
                    } else if let pi = pendingI {
                        aligned.append(pi.0); aligned.append(pi.1)
                        aligned.append(lo2); aligned.append(hi2)
                        pendingI = nil
                        expectingI = true
                    }
                    continue
                }
            }

            if expectingI {
                pendingI = (lo, hi)
                expectingI = false
            } else if let pi = pendingI {
                aligned.append(pi.0); aligned.append(pi.1)
                aligned.append(lo); aligned.append(hi)
                pendingI = nil
                expectingI = true
            }
        }

        if !aligned.isEmpty {
            let dropped = aligned.withUnsafeBytes { sampleFIFO.write($0) }
            if dropped {
                droppedSinceLastDrain = true
            }
        }
    }

    // Drains up to `maxBytes` of aligned interleaved int16 I/Q from the FIFO
    // directly into the caller-owned `outBuffer` (must be at least
    // `maxBytes` bytes). Always drains a whole number of (I, Q) pairs
    // (multiples of 4 bytes). Returns the number of bytes actually written,
    // which may be less than `maxBytes` (including zero) if the FIFO is
    // drained dry.
    //
    // `sampleFIFO` is now a fixed-capacity ring buffer (see ByteRingBuffer),
    // so this is a plain wrapped memcpy with no allocation and no backing-
    // store growth of any kind -- fixing both the earlier `Data`-based
    // allocation churn AND the subtler `Data.removeFirst()` non-compaction
    // bug that caused steady MALLOC_REALLOC growth in vmmap despite the
    // FIFO's logical byte count always staying correctly capped.
    //
    // `outDropped` is set to `true` if the FIFO overflowed and dropped the
    // oldest samples since the last call to this function, so a
    // SoapySDR-style caller can report exactly one overflow indication per
    // drop event.
    func drain(into outBuffer: UnsafeMutableRawPointer, maxBytes: Int, outDropped: inout Bool) -> Int {
        queue.sync {
            outDropped = droppedSinceLastDrain
            droppedSinceLastDrain = false

            let takeAligned = min(maxBytes, sampleFIFO.occupied) / 4 * 4
            guard takeAligned > 0 else { return 0 }

            return sampleFIFO.read(into: outBuffer, maxBytes: takeAligned)
        }
    }

    func snap() -> (Int, Int, Int) {
        queue.sync { (totalBytes, syncCount, completions) }
    }

    func stop() {
        queue.sync { stopping = true }
    }
}

private func propInt(_ s: io_service_t, _ k: String) -> Int? {
    guard let cf = IORegistryEntryCreateCFProperty(s, k as CFString, kCFAllocatorDefault, 0) else { return nil }
    return (cf.takeRetainedValue() as? NSNumber)?.intValue
}

private func findDevice(pid: Int) -> io_service_t {
    var it: io_iterator_t = 0
    IOServiceGetMatchingServices(kIOMainPortDefault, IOServiceMatching("IOUSBHostDevice"), &it)
    var chosen: io_service_t = 0
    while case let s = IOIteratorNext(it), s != 0 {
        if propInt(s, "idVendor") == VID, propInt(s, "idProduct") == pid, chosen == 0 {
            chosen = s
        } else {
            IOObjectRelease(s)
        }
    }
    IOObjectRelease(it)
    return chosen
}

private func findInterfaceService() -> io_service_t {
    var it: io_iterator_t = 0
    IOServiceGetMatchingServices(kIOMainPortDefault, IOServiceMatching("IOUSBHostInterface"), &it)
    var chosen: io_service_t = 0
    while case let s = IOIteratorNext(it), s != 0 {
        let ok = propInt(s, "idVendor") == VID && propInt(s, "idProduct") == PID_STREAM && propInt(s, "bInterfaceNumber") == 0
        if ok, chosen == 0 {
            chosen = s
        } else {
            IOObjectRelease(s)
        }
    }
    IOObjectRelease(it)
    return chosen
}

private struct FirmwareRecord: Decodable {
    let wValue: Int
    let data: String
}

private func hexBytes(_ hex: String) -> [UInt8] {
    var out = [UInt8]()
    var i = hex.startIndex
    while i < hex.endIndex {
        let j = hex.index(i, offsetBy: 2)
        out.append(UInt8(hex[i..<j], radix: 16)!)
        i = j
    }
    return out
}

private func frame(_ payload: [UInt8]) -> [UInt8] {
    var f: [UInt8] = [0xFE, 0xFE, 0x96, 0xE0]
    f += payload
    f.append(0xFD)
    if f.count % 2 != 0 { f.append(0xFF) }
    return f
}

private func freqBCD(_ hz: UInt64) -> [UInt8] {
    var d = [UInt8](repeating: 0, count: 10)
    var v = hz
    for i in 0..<10 {
        d[i] = UInt8(v % 10)
        v /= 10
    }
    return (0..<5).map { (d[$0 * 2 + 1] << 4) | d[$0 * 2] }
}

// Attenuator dB -> CI-V byte (0/10/20/30 dB -> 0x00/0x10/0x20/0x30), snapped
// to the nearest supported step.
private func attBCD(_ db: Double) -> UInt8 {
    let steps: [(Double, UInt8)] = [(0, 0x00), (10, 0x10), (20, 0x20), (30, 0x30)]
    return steps.min(by: { abs($0.0 - db) < abs($1.0 - db) })!.1
}

// RF gain 0..255 -> two BCD bytes (hi, lo), e.g. 255 -> [0x02, 0x55].
private func rfGainBCD(_ value: Double) -> [UInt8] {
    let clamped = max(0, min(255, Int(value.rounded())))
    func bcdByte(_ v: Int) -> UInt8 { UInt8(((v / 10) % 10) << 4 | (v % 10)) }
    let hi = bcdByte(clamped / 100)
    let lo = bcdByte(clamped % 100)
    return [hi, lo]
}


private func loadFirmwareSequence(from path: String) -> [FirmwareRecord]? {
    guard let data = try? Data(contentsOf: URL(fileURLWithPath: path)),
          let seq = try? JSONDecoder().decode([FirmwareRecord].self, from: data) else {
        return nil
    }
    return seq
}

private func downloadFirmwareAndWaitForReenumeration(loaderService: io_service_t, sequence: [FirmwareRecord]) -> io_service_t {
    guard let dev = try? IOUSBHostDevice(__ioService: loaderService, options: [], queue: nil, interestHandler: nil) else {
        return 0
    }

    for r in sequence {
        let payload = hexBytes(r.data)
        let req = IOUSBDeviceRequest(bmRequestType: 0x40, bRequest: 0xA0,
                                     wValue: UInt16(r.wValue), wIndex: 0, wLength: UInt16(payload.count))
        let d = NSMutableData(bytes: payload, length: payload.count)
        var got: UInt = 0
        do {
            try dev.__send(req, data: d, bytesTransferred: &got, completionTimeout: 2.0)
        } catch {
            // expected around final reset/re-enumeration edge
        }
        usleep(3000)
    }

    for _ in 0..<40 {
        usleep(250_000)
        let streamService = findDevice(pid: PID_STREAM)
        if streamService != 0 { return streamService }
    }
    return 0
}

// Returns (success, errorDescription). Errors are returned rather than
// printed here -- transient settle-timing failures that a retry clears are
// not worth logging; sendCIVCommand only logs if all attempts are exhausted.
private func sendCIVCommandOnce(box: ICR8600DeviceBox, payload: [UInt8]) -> (Bool, String?) {
    guard let outPipe = box.outPipe, let ackPipe = box.ackPipe else { return (false, "no pipe") }
    let f = frame(payload)
    let out = NSMutableData(bytes: f, length: f.count)
    var sent: UInt = 0
    do {
        try outPipe.__sendIORequest(with: out, bytesTransferred: &sent, completionTimeout: 1.5)
    } catch {
        return (false, "send: \(error)")
    }

    let ack = NSMutableData(length: 64)!
    var got: UInt = 0
    do {
        try ackPipe.__sendIORequest(with: ack, bytesTransferred: &got, completionTimeout: 1.5)
        return (true, nil)
    } catch {
        return (false, "ack: \(error)")
    }
}

// CI-V sends can rarely hit a low-frequency kernel/USB timing race
// ("Unable to send IO"). A short retry clears it reliably, so only log if
// all attempts are exhausted; a transient that recovers immediately is not
// worth console noise on the normal path.
private func sendCIVCommand(box: ICR8600DeviceBox, payload: [UInt8]) -> Bool {
    var lastError: String?
    for attempt in 0..<3 {
        let (ok, err) = sendCIVCommandOnce(box: box, payload: payload)
        if ok { return true }
        lastError = err
        if attempt < 2 { usleep(100_000) }
    }
    print("CI-V command failed after 3 attempts: \(lastError ?? "unknown error")")
    return false
}

private func getBox(_ raw: UnsafeMutableRawPointer?) -> ICR8600DeviceBox? {
    guard let raw else { return nil }
    return Unmanaged<ICR8600DeviceBox>.fromOpaque(raw).takeUnretainedValue()
}

@_cdecl("icr8600_open")
public func icr8600_open(_ firmwarePath: UnsafePointer<CChar>?, _ outDevice: UnsafeMutablePointer<UnsafeMutableRawPointer?>?) -> Int32 {
    guard let outDevice = outDevice else { return ICR8600_ERR_IO.rawValue }

    let existingStream = findDevice(pid: PID_STREAM)
    if existingStream != 0 {
        guard let hostDevice = try? IOUSBHostDevice(__ioService: existingStream, options: [], queue: nil, interestHandler: nil) else {
            IOObjectRelease(existingStream)
            return ICR8600_ERR_IO.rawValue
        }
        let box = ICR8600DeviceBox(ioService: existingStream, hostDevice: hostDevice)
        outDevice.pointee = UnsafeMutableRawPointer(Unmanaged.passRetained(box).toOpaque())
        return ICR8600_OK.rawValue
    }

    let loaderService = findDevice(pid: PID_LOADER)
    guard loaderService != 0 else { return ICR8600_ERR_NOT_FOUND.rawValue }
    guard let firmwarePath = firmwarePath else {
        IOObjectRelease(loaderService)
        return ICR8600_ERR_FIRMWARE_FILE_NOT_FOUND.rawValue
    }
    let pathString = String(cString: firmwarePath)
    guard let sequence = loadFirmwareSequence(from: pathString) else {
        IOObjectRelease(loaderService)
        return ICR8600_ERR_FIRMWARE_FILE_NOT_FOUND.rawValue
    }

    let streamService = downloadFirmwareAndWaitForReenumeration(loaderService: loaderService, sequence: sequence)
    IOObjectRelease(loaderService)
    guard streamService != 0 else { return ICR8600_ERR_FIRMWARE_LOAD_FAILED.rawValue }

    guard let hostDevice = try? IOUSBHostDevice(__ioService: streamService, options: [], queue: nil, interestHandler: nil) else {
        IOObjectRelease(streamService)
        return ICR8600_ERR_IO.rawValue
    }
    let box = ICR8600DeviceBox(ioService: streamService, hostDevice: hostDevice)
    outDevice.pointee = UnsafeMutableRawPointer(Unmanaged.passRetained(box).toOpaque())
    return ICR8600_OK.rawValue
}

@_cdecl("icr8600_close")
public func icr8600_close(_ device: UnsafeMutableRawPointer?) {
    icr8600_stop_streaming(device)
    guard let device else { return }
    Unmanaged<ICR8600DeviceBox>.fromOpaque(device).release()
}

// rateCode: 0x01=5.12MS/s 0x02=3.84 0x03=1.92 0x04=960k 0x05=480k 0x06=240k
// (per the documented device protocol). frequencyHz is the
// initial tuned center frequency; both are now real parameters instead of
// the hardcoded 5.12 MS/s @ 7.100 MHz used for the first smoke test.
@_cdecl("icr8600_start_streaming")
public func icr8600_start_streaming(_ device: UnsafeMutableRawPointer?, _ rateCode: UInt8, _ frequencyHz: UInt64) -> Int32 {
    guard let box = getBox(device) else { return ICR8600_ERR_IO.rawValue }
    if box.isStreaming { return ICR8600_OK.rawValue }

    do {
        try box.hostDevice.__configure(withValue: 1, matchInterfaces: true)
    } catch {
        print("configure failed: \(error)")
        return ICR8600_ERR_IO.rawValue
    }

    var ifSvc: io_service_t = 0
    for _ in 0..<60 {
        ifSvc = findInterfaceService()
        if ifSvc != 0 { break }
        usleep(50_000)
    }
    guard ifSvc != 0 else {
        print("interface 0 not found")
        return ICR8600_ERR_IO.rawValue
    }

    let ioQueue = DispatchQueue(label: "icr8600.iq")
    do {
        let iface = try IOUSBHostInterface(__ioService: ifSvc, options: [], queue: ioQueue, interestHandler: nil)
        try iface.selectAlternateSetting(0)
        // Copy all three pipes BEFORE touching `box` state at all. The
        // previous version assigned `box.interfaceService`/`interfaceObject`
        // immediately after opening the interface, then copied pipes one at
        // a time -- so a failure partway through copyPipe left `box` holding
        // a live, exclusively-open IOUSBHostInterface with only some pipes
        // set, and the catch block below only released the raw `ifSvc`
        // io_service_t, never `box.interfaceObject` itself. That interface
        // claim then outlived this function call, blocking every subsequent
        // start_streaming attempt with "Failed to create IOUSBHostInterface"
        // until the process exited. Assigning to `box` only after all three
        // copyPipe calls succeed means any exception here leaves `box`
        // completely untouched, and `iface`/`ifSvc` simply fall out of scope
        // (releasing the interface) once we hit the catch block.
        let out = try iface.copyPipe(withAddress: 0x02)
        let ack = try iface.copyPipe(withAddress: 0x88)
        let iq = try iface.copyPipe(withAddress: 0x86)
        box.interfaceService = ifSvc
        box.interfaceObject = iface
        box.outPipe = out
        box.ackPipe = ack
        box.iqPipe = iq
    } catch {
        IOObjectRelease(ifSvc)
        print("interface/pipe setup failed: \(error)")
        return ICR8600_ERR_IO.rawValue
    }

    // Give the kernel a moment to finish settling the bulk pipes after
    // selectAlternateSetting/copyPipe before the first CI-V transfer —
    // sending immediately can intermittently fail with "Unable to send IO."
    // Note: in practice the first CI-V command still hits this failure and
    // falls through to sendCIVCommand's built-in retry almost every time,
    // regardless of how long this delay is (tested up to 250ms with no
    // improvement) -- this appears to be a one-shot kernel/pipe state
    // transition triggered by the first failed attempt itself, not
    // something a longer fixed sleep can pre-empt. The retry-with-backoff
    // in sendCIVCommand is the real fix; this delay is kept at a modest
    // value just to avoid needlessly guaranteeing that first failure.
    usleep(50_000)

    let startup: [[UInt8]] = [
        [0x1A, 0x13, 0x01, 0x00],
        [0x1A, 0x13, 0x00, 0x01],
        [0x05] + freqBCD(frequencyHz),
        [0x11, attBCD(box.attDb)],
        [0x12, box.antennaIndex],
        [0x14, 0x02] + rfGainBCD(box.rfGain),
        [0x16, 0x02, box.preampOn > 0.5 ? 0x01 : 0x00],
        [0x16, 0x65, box.ippOn > 0.5 ? 0x01 : 0x00],
        [0x1A, 0x13, 0x02, 0x01],
        [0x1A, 0x13, 0x01, 0x01, 0x00, rateCode]
    ]
    for payload in startup {
        if !sendCIVCommand(box: box, payload: payload) {
            // STUCK-INTERFACE FIX: the interface and all three pipes above
            // are already open and exclusively claimed at this point. If we
            // just returned here without releasing them, `box.isStreaming`
            // would stay false but `box.interfaceObject`/pipes would stay
            // alive, still holding the exclusive claim -- so the NEXT
            // start_streaming call would find the interface already open
            // (by this same leaked object) and fail immediately with
            // "Failed to create IOUSBHostInterface." This is exactly what
            // happens if the CI-V startup handshake fails right after a
            // stop/start cycle: the first restart fails here, and the
            // interface stays wedged open for every attempt after that
            // until the process is killed. Tear everything down the same
            // way icr8600_stop_streaming does before reporting the error,
            // so a retry from a clean state is possible.
            box.iqPipe = nil
            box.ackPipe = nil
            box.outPipe = nil
            box.interfaceObject = nil
            if box.interfaceService != 0 {
                IOObjectRelease(box.interfaceService)
                box.interfaceService = 0
            }
            return ICR8600_ERR_IO.rawValue
        }
    }

    if let iqPipe = box.iqPipe {
        let reader = AsyncReader(pipe: iqPipe, queue: ioQueue, nbuf: NBUF, size: BUFSIZE)
        box.reader = reader
        reader.start()
        box.frequencyHz = frequencyHz
        box.isStreaming = true
        print("IC-R8600 stream started: rateCode=0x\(String(rateCode, radix: 16)) freq=\(frequencyHz) Hz")
        return ICR8600_OK.rawValue
    }
    return ICR8600_ERR_IO.rawValue
}

@_cdecl("icr8600_get_stream_stats")
public func icr8600_get_stream_stats(_ device: UnsafeMutableRawPointer?, _ outBytes: UnsafeMutablePointer<UInt64>?, _ outSyncs: UnsafeMutablePointer<UInt64>?, _ outCompletions: UnsafeMutablePointer<UInt64>?) -> Int32 {
    guard let box = getBox(device), let reader = box.reader else { return ICR8600_ERR_IO.rawValue }
    let (bytes, syncs, completions) = reader.snap()
    outBytes?.pointee = UInt64(bytes)
    outSyncs?.pointee = UInt64(syncs)
    outCompletions?.pointee = UInt64(completions)
    return ICR8600_OK.rawValue
}

// Drains up to `maxSamplePairs` aligned (I, Q) int16 pairs into outBuffer
// (caller-allocated, at least maxSamplePairs*2 int16s / maxSamplePairs*4
// bytes) via a direct copy from the internal FIFO -- no intermediate Data
// allocation or extra memcpy. outPairsWritten receives the actual number
// of pairs written, which may be less than requested (or zero) if the
// FIFO is drained dry.
//
// outDropped, if non-NULL, is set to 1 if the internal FIFO overflowed and
// dropped the oldest samples since the previous call to this function, or
// 0 otherwise. Callers should surface this as a stream overflow indicator
// (e.g. SOAPY_SDR_OVERFLOW) on the read immediately following a drop.
@_cdecl("icr8600_read_iq")
public func icr8600_read_iq(_ device: UnsafeMutableRawPointer?, _ outBuffer: UnsafeMutableRawPointer?, _ maxSamplePairs: Int, _ outPairsWritten: UnsafeMutablePointer<Int>?, _ outDropped: UnsafeMutablePointer<Int32>?) -> Int32 {
    guard let box = getBox(device), let reader = box.reader, let outBuffer, maxSamplePairs > 0 else {
        outPairsWritten?.pointee = 0
        outDropped?.pointee = 0
        return ICR8600_ERR_IO.rawValue
    }
    let maxBytes = maxSamplePairs * 4
    var dropped = false
    let bytesWritten = reader.drain(into: outBuffer, maxBytes: maxBytes, outDropped: &dropped)
    outPairsWritten?.pointee = bytesWritten / 4
    outDropped?.pointee = dropped ? 1 : 0
    return ICR8600_OK.rawValue
}

@_cdecl("icr8600_stop_streaming")
@discardableResult
public func icr8600_stop_streaming(_ device: UnsafeMutableRawPointer?) -> Int32 {
    guard let box = getBox(device) else { return ICR8600_ERR_IO.rawValue }
    if !box.isStreaming { return ICR8600_OK.rawValue }

    box.reader?.stop()
    box.reader = nil

    _ = sendCIVCommand(box: box, payload: [0x1A, 0x13, 0x01, 0x00])
    _ = sendCIVCommand(box: box, payload: [0x1A, 0x13, 0x00, 0x00])

    box.iqPipe = nil
    box.ackPipe = nil
    box.outPipe = nil
    box.interfaceObject = nil
    if box.interfaceService != 0 {
        IOObjectRelease(box.interfaceService)
        box.interfaceService = 0
    }
    box.isStreaming = false
    print("IC-R8600 stream stopped.")
    return ICR8600_OK.rawValue
}

// Live gain control. If streaming, sends the corresponding CI-V command
// immediately (all of ATT/RF/preamp/IP+ change live per the protocol
// notes -- only sample-rate/bit-depth require a full OFF->ON cycle).
// If not streaming, only updates the cached value used on next start.
@_cdecl("icr8600_set_gain")
public func icr8600_set_gain(_ device: UnsafeMutableRawPointer?, _ element: UInt8, _ value: Double) -> Int32 {
    guard let box = getBox(device) else { return ICR8600_ERR_IO.rawValue }

    var payload: [UInt8]
    switch UInt32(element) {
    case ICR8600_GAIN_ATT.rawValue:
        box.attDb = value
        payload = [0x11, attBCD(value)]
    case ICR8600_GAIN_RF.rawValue:
        // Cache the same clamped/rounded integer that rfGainBCD() actually
        // encodes and sends, not the raw requested double -- otherwise
        // getGain(RF) can report a value the radio was never actually set
        // to (e.g. a fractional request gets rounded for the wire but the
        // unrounded value stays cached), which then disagrees with any
        // other UI control mirroring the same reading (e.g. SDRangel's
        // Global Gain, which does its own separate rounding for display).
        let clampedRF = Double(max(0, min(255, Int(value.rounded()))))
        box.rfGain = clampedRF
        payload = [0x14, 0x02] + rfGainBCD(clampedRF)
    case ICR8600_GAIN_PREAMP.rawValue:
        box.preampOn = value
        payload = [0x16, 0x02, value > 0.5 ? 0x01 : 0x00]
    case ICR8600_GAIN_IPP.rawValue:
        box.ippOn = value
        payload = [0x16, 0x65, value > 0.5 ? 0x01 : 0x00]
    default:
        return ICR8600_ERR_IO.rawValue
    }

    if box.isStreaming {
        if !sendCIVCommand(box: box, payload: payload) { return ICR8600_ERR_IO.rawValue }
    }
    return ICR8600_OK.rawValue
}

@_cdecl("icr8600_get_gain")
public func icr8600_get_gain(_ device: UnsafeMutableRawPointer?, _ element: UInt8, _ outValue: UnsafeMutablePointer<Double>?) -> Int32 {
    guard let box = getBox(device), let outValue else { return ICR8600_ERR_IO.rawValue }
    switch UInt32(element) {
    case ICR8600_GAIN_ATT.rawValue: outValue.pointee = box.attDb
    case ICR8600_GAIN_RF.rawValue: outValue.pointee = box.rfGain
    case ICR8600_GAIN_PREAMP.rawValue: outValue.pointee = box.preampOn
    case ICR8600_GAIN_IPP.rawValue: outValue.pointee = box.ippOn
    default: return ICR8600_ERR_IO.rawValue
    }
    return ICR8600_OK.rawValue
}

// Sets the tuned center frequency (Hz). Updates the device's cached
// frequency immediately; if streaming is currently active, also sends the
// CI-V frequency command (0x05) live -- frequency changes live per the
// reverse-engineered protocol notes, same as the gain elements, and do NOT
// require a stream restart. This matters in practice: several SoapySDR
// client UIs (e.g. digit-by-digit frequency entry widgets) call setFrequency
// once per keystroke/click, and a full stop/start cycle per call was both
// slow and guaranteed to hit the first-CI-V-command settle failure on every
// single click. If not currently streaming, the value is cached and used on
// the next icr8600_start_streaming call.
@_cdecl("icr8600_set_frequency")
public func icr8600_set_frequency(_ device: UnsafeMutableRawPointer?, _ frequencyHz: UInt64) -> Int32 {
    guard let box = getBox(device) else { return ICR8600_ERR_IO.rawValue }
    box.frequencyHz = frequencyHz
    if box.isStreaming {
        if !sendCIVCommand(box: box, payload: [0x05] + freqBCD(frequencyHz)) { return ICR8600_ERR_IO.rawValue }
    }
    return ICR8600_OK.rawValue
}

// Reads back the cached frequency (no hardware round-trip).
@_cdecl("icr8600_get_frequency")
public func icr8600_get_frequency(_ device: UnsafeMutableRawPointer?, _ outFrequencyHz: UnsafeMutablePointer<UInt64>?) -> Int32 {
    guard let box = getBox(device) else { return ICR8600_ERR_IO.rawValue }
    outFrequencyHz?.pointee = box.frequencyHz
    return ICR8600_OK.rawValue
}

@_cdecl("icr8600_set_antenna")
public func icr8600_set_antenna(_ device: UnsafeMutableRawPointer?, _ index: UInt8) -> Int32 {
    guard let box = getBox(device) else { return ICR8600_ERR_IO.rawValue }
    box.antennaIndex = index
    if box.isStreaming {
        if !sendCIVCommand(box: box, payload: [0x12, index]) { return ICR8600_ERR_IO.rawValue }
    }
    return ICR8600_OK.rawValue
}

@_cdecl("icr8600_get_antenna")
public func icr8600_get_antenna(_ device: UnsafeMutableRawPointer?, _ outIndex: UnsafeMutablePointer<UInt8>?) -> Int32 {
    guard let box = getBox(device), let outIndex else { return ICR8600_ERR_IO.rawValue }
    outIndex.pointee = box.antennaIndex
    return ICR8600_OK.rawValue
}
