// cw_reader.cpp - read lines from 8 CosmicWatch detectors (USB serial,
// dev/CW0 .. dev/CW7 = symlinks to /dev/serial/by-id/...) with one epoll loop,
// timestamp every event and write the result to a file or to the terminal.
//
// Build: g++ -std=c++17 -O2 -Wall -Wextra -Wno-psabi -pthread cw_reader.cpp -o cw_reader
//
// Version 20261117 (based on 20261105):
//  - Time zero of the readout ("start") is the moment when ALL opened detectors
//    have sent their header + name, i.e. are known to be running (at most
//    kStartTimeoutNs = 10 s after the ports were opened; detectors still silent
//    then are reported and join later). Data lines received before the start
//    are discarded; event times and the running time count from the start.
//  - Reduced terminal output when a running time is given (scheduled runs):
//    detector names in one line, event counters every 10 min instead of every
//    minute, no "closing ... (shutdown)" lines, no repeated final counters.
// Version 20261117b:
//  - Silent-detector check: if a detector sends no data for more than
//    kGapWarnNs (30 s), a warning is printed at once (checked once per second,
//    not per event); when it sends again, the length of the gap is printed.
//    At the end a summary of all gaps > 30 s per detector is printed.
//    Per event this costs one store and one flag test.
//
// Usage: cw_reader [output_file|-] [seconds] [baud]
//   cw_reader                        events are printed to the terminal, stop with Ctrl-C
//   cw_reader events.txt             events are appended to events.txt, stop with Ctrl-C
//   cw_reader events.txt 3600        same, but stops by itself after 3600 s
//   cw_reader - 60                   terminal output, stops after 60 s
//   cw_reader events.txt 0 115200    no time limit (0), serial ports opened at 115200 baud
// The baud rate (default 9600) must match Serial.begin() in the detector firmware,
// otherwise only garbage is received.
//
// Output line:  <detector_number> <time_since_start_us> <line from the detector>
// The detector's line is stored unchanged (only CR/LF removed). The time is
// measured with CLOCK_MONOTONIC from the start (all detectors ready, see above).
// It is the time of the epoll wake-up that delivered the FIRST byte of the
// line. (At 9600 baud a ~45 character line takes ~47 ms to arrive in several
// pieces; stamping the piece with the final '\n' made the time depend on the
// line length by ~1.04 ms per character.)
//
// Every detector sends a 5-line header ('###...') followed by a line with its
// unique name when the connection is opened. The header is discarded, the name
// is printed to the terminal (stderr) and only the following lines are stored.
// Once the name has been received a detector's lines are not inspected any
// more (so a header repeated after a reset of the detector would be stored too).
//
// stdout carries data only. Names, event counters and errors go to stderr.

#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/signalfd.h>
#include <linux/serial.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------- settings
static constexpr int kNumDetectors = 8;      // /dev/ttyUSB0 .. /dev/ttyUSB7
static constexpr long kDefaultBaud = 9600;   // must match Serial.begin() in the firmware

// Standard Linux baud rates -> termios constants. Returns false if not supported.
static bool baud_to_speed(long baud, speed_t& sp) {
    static const struct { long b; speed_t s; } table[] = {
        {1200, B1200}, {2400, B2400}, {4800, B4800}, {9600, B9600}, {19200, B19200},
        {38400, B38400}, {57600, B57600}, {115200, B115200}, {230400, B230400},
        {460800, B460800}, {500000, B500000}, {576000, B576000}, {921600, B921600},
        {1000000, B1000000}, {1152000, B1152000}, {1500000, B1500000}, {2000000, B2000000},
    };
    for (const auto& e : table)
        if (e.b == baud) { sp = e.s; return true; }
    return false;
}
static constexpr int kMaxEvents = 16;        // >= number of fds in epoll
static constexpr uint32_t kSignalTag = UINT32_MAX;
static constexpr size_t kMaxPartialLine = 4096;

static constexpr int64_t kNsPerSec = 1'000'000'000LL;
static constexpr int64_t kStatusIntervalNs = 60 * kNsPerSec;        // event counters every minute
static constexpr int64_t kStatusIntervalQuietNs = 600 * kNsPerSec;  // ... every 10 min with a running time
static constexpr int64_t kStartTimeoutNs = 10 * kNsPerSec;          // max. wait for all detector headers
static constexpr int64_t kGapWarnNs = 30 * kNsPerSec;               // warn if a detector is silent longer

// ---------------------------------------------------------------- time
// CLOCK_MONOTONIC: never jumps when NTP steps the wall clock, and its rate is
// corrected by NTP, so intervals stay true seconds.
static inline int64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * kNsPerSec + ts.tv_nsec;
}

static std::string hms(int64_t seconds) {
    char b[32];
    std::snprintf(b, sizeof b, "%02lld:%02lld:%02lld", (long long)(seconds / 3600),
                  (long long)(seconds / 60 % 60), (long long)(seconds % 60));
    return b;
}

static std::string wall_stamp() {
    time_t wall = time(nullptr);
    tm lt;
    char b[64] = "?";
    if (localtime_r(&wall, &lt)) strftime(b, sizeof b, "%Y-%m-%d %H:%M:%S", &lt);
    return b;
}

// ---------------------------------------------------------------- queue to writer thread
struct Record {
    uint8_t id;
    int64_t time_us;     // microseconds since the start (all detectors ready)
    std::string line;    // data line exactly as sent by the detector (CR/LF removed)
};

class Queue {
public:
    // Hands over everything collected during one epoll wakeup: one lock and one
    // wake-up of the writer per batch instead of per event.
    void push_all(std::vector<Record>& v) {
        if (v.empty()) return;
        {
            std::lock_guard<std::mutex> lk(m_);
            q_.insert(q_.end(), std::make_move_iterator(v.begin()), std::make_move_iterator(v.end()));
        }
        cv_.notify_one();
        v.clear();
    }
    // Blocks until data or finish(). Returns false when finished and drained.
    bool pop_all(std::vector<Record>& out) {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&] { return !q_.empty() || done_; });
        out.clear();
        out.swap(q_);
        return !out.empty() || !done_;
    }
    void finish() {
        {
            std::lock_guard<std::mutex> lk(m_);
            done_ = true;
        }
        cv_.notify_all();
    }

private:
    std::mutex m_;
    std::condition_variable cv_;
    std::vector<Record> q_;
    bool done_ = false;
};

// Writes to any std::ostream: an ofstream or std::cout.
static void writer_thread(Queue& q, std::ostream& out) {
    std::vector<Record> batch;
    while (q.pop_all(batch)) {
        for (const Record& r : batch)
            out << int(r.id) << ' ' << r.time_us << ' ' << r.line << '\n';
        out.flush();   // also needed for cout when stdout is a pipe (fully buffered)
    }
    out.flush();
}

// ---------------------------------------------------------------- per-detector state
// A detector sends: header lines ('###...'), then one line with its name,
// then data lines. WaitHeader drops anything received before the first header
// (e.g. the tail of a line cut in half when the port was opened).
enum class State { WaitHeader, AfterHeader, Data };

struct Port {
    int fd = -1;
    std::string path;
    std::string partial;       // incomplete line carried over between reads
    int64_t partial_t_us = 0;  // time of the wake-up that delivered the first byte of 'partial'
    State state = State::WaitHeader;
    uint64_t events = 0;       // data lines stored so far
    std::string name;          // detector name (line after the header)
    // silent-detector bookkeeping (times in us since the start)
    int64_t last_us = 0;       // time of the last stored event (or of the start / joining)
    bool silent = false;       // no data for more than kGapWarnNs, warning printed
    uint32_t n_gaps = 0;       // finished + ongoing gaps > kGapWarnNs
    int64_t gap_total_us = 0;  // summed length of finished gaps
    int64_t gap_max_us = 0;    // longest gap (finished or ongoing at the end)
};

// Gap of detector p ends (it sent data again at t_us, or the run ends): book it.
static void close_gap(Port& p, int64_t t_us) {
    const int64_t g = t_us - p.last_us;
    ++p.n_gaps;
    p.gap_total_us += g;
    if (g > p.gap_max_us) p.gap_max_us = g;
    p.silent = false;
}

// Header phase only: runs a few times per detector at start-up, never once
// the state is Data, so it may be as slow as it likes.
static void handle_header_line(int id, Port& p, const char* b, const char* e, bool print) {
    if (e - b >= 3 && b[0] == '#' && b[1] == '#' && b[2] == '#') {
        p.state = State::AfterHeader;                 // header line: discard
        return;
    }
    if (p.state == State::WaitHeader) return;         // junk before the header: discard
    // first line after the header = unique name of the detector
    p.name.assign(b, e);
    if (print) std::fprintf(stderr, "Detector %d (%s): %s\n", id, p.path.c_str(), p.name.c_str());
    p.state = State::Data;
}

// Cuts the bytes read from a port into lines and calls on_line(begin, end, t_us)
// for each complete line (without '\n'). t_us is the time of the wake-up that
// delivered the line's first byte: now_us for a line starting in this read,
// the remembered p.partial_t_us for a line continued from an earlier read.
// A line lying completely inside the read buffer is passed on in place, with
// no copy; only a line cut by the end of a read goes through p.partial.
template <class F>
static void split_lines(Port& p, const char* data, size_t len, int64_t now_us, F&& on_line) {
    const char* const end = data + len;
    while (data < end) {
        const char* nl = static_cast<const char*>(std::memchr(data, '\n', size_t(end - data)));
        if (!nl) {                                    // incomplete line: keep for the next read
            if (p.partial.empty()) p.partial_t_us = now_us;   // its first byte arrived now
            p.partial.append(data, size_t(end - data));
            if (p.partial.size() > kMaxPartialLine) p.partial.clear();   // garbage guard
            return;
        }
        if (p.partial.empty()) {
            on_line(data, nl, now_us);
        } else {
            p.partial.append(data, size_t(nl - data));
            on_line(p.partial.data(), p.partial.data() + p.partial.size(), p.partial_t_us);
            p.partial.clear();
        }
        data = nl + 1;
    }
}

static void print_status(const std::vector<Port>& ports, int64_t elapsed_ns) {
    std::string s;
    uint64_t total = 0;
    for (size_t i = 0; i < ports.size(); ++i) {
        s += " d" + std::to_string(i) + "=" + std::to_string(ports[i].events);
        total += ports[i].events;
    }
    std::fprintf(stderr, "[%s] events:%s  total=%llu\n", hms(elapsed_ns / kNsPerSec).c_str(),
                 s.c_str(), (unsigned long long)total);
}

// ---------------------------------------------------------------- serial port
static int open_serial(const char* path, speed_t speed) {
    // O_NONBLOCK: read() never blocks, so epoll decides when we read.
    int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        std::fprintf(stderr, "open %s: %s\n", path, std::strerror(errno));
        return -1;
    }

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        std::fprintf(stderr, "tcgetattr %s: %s\n", path, std::strerror(errno));
        ::close(fd);
        return -1;
    }
    cfmakeraw(&tio);                       // no echo, no line discipline, no CR/LF mangling
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);       // ignore modem lines, enable receiver
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        std::fprintf(stderr, "tcsetattr %s: %s\n", path, std::strerror(errno));
        ::close(fd);
        return -1;
    }

    // tcsetattr() succeeds if ANY setting was applied: read back and check the speed
    termios chk{};
    if (tcgetattr(fd, &chk) != 0 || cfgetispeed(&chk) != speed || cfgetospeed(&chk) != speed) {
        std::fprintf(stderr, "%s: requested baud rate not accepted by the driver\n", path);
        ::close(fd);
        return -1;
    }

    ioctl(fd, TIOCEXCL);                   // refuse other openers (best effort)

    serial_struct ss;                      // ask the driver for minimal buffering latency
    if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
        ss.flags |= ASYNC_LOW_LATENCY;
        ioctl(fd, TIOCSSERIAL, &ss);       // not supported by every USB-serial driver; ignore failure
    }

    tcflush(fd, TCIOFLUSH);                // drop anything buffered before we started
    return fd;
}

// ---------------------------------------------------------------- epoll helpers
static bool epoll_add(int epfd, int fd, uint32_t tag) {
    epoll_event ev{};
    ev.events = EPOLLIN;                   // EPOLLERR / EPOLLHUP are always reported
    ev.data.u32 = tag;                     // we use the port index as the tag
    return epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) == 0;
}

static void drop_port(int epfd, Port& p, int& open_ports, const char* why, bool print = true) {
    if (p.fd < 0) return;
    if (print) std::fprintf(stderr, "closing %s (%s)\n", p.path.c_str(), why);
    epoll_ctl(epfd, EPOLL_CTL_DEL, p.fd, nullptr);   // explicit remove, then close
    ::close(p.fd);
    p.fd = -1;
    --open_ports;
}

// ---------------------------------------------------------------- main
static int usage(const char* prog) {
    std::fprintf(stderr,
                 "usage: %s [output_file|-] [seconds] [baud]\n"
                 "  no file name or '-' : events are printed to the terminal\n"
                 "  seconds             : stop after this running time, counted from the moment all\n"
                 "                        detectors are ready; 0 or omitted = run until Ctrl-C.\n"
                 "                        With a running time the terminal output is reduced.\n"
                 "  baud                : serial speed, default %ld; must match Serial.begin() in the firmware\n"
                 "                        (1200 2400 4800 9600 19200 38400 57600 115200 230400 460800\n"
                 "                         500000 576000 921600 1000000 1152000 1500000 2000000)\n",
                 prog, kDefaultBaud);
    return 1;
}

int main(int argc, char** argv) {
    if (argc > 4) return usage(argv[0]);

    // Optional running time.
    double duration_s = 0;                 // 0 = no limit
    if (argc >= 3) {
        char* endp = nullptr;
        duration_s = std::strtod(argv[2], &endp);
        if (endp == argv[2] || *endp != '\0' || !(duration_s >= 0) || duration_s > 1e9) {
            std::fprintf(stderr, "invalid running time '%s'\n", argv[2]);
            return usage(argv[0]);
        }
    }

    // Optional baud rate.
    long baud = kDefaultBaud;
    speed_t speed = B9600;
    if (argc == 4) {
        char* endp = nullptr;
        baud = std::strtol(argv[3], &endp, 10);
        if (endp == argv[3] || *endp != '\0' || !baud_to_speed(baud, speed)) {
            std::fprintf(stderr, "unsupported baud rate '%s'\n", argv[3]);
            return usage(argv[0]);
        }
    }
    const bool has_duration = duration_s > 0;
    const int64_t duration_ns = int64_t(duration_s * double(kNsPerSec));
    const bool quiet = has_duration;       // reduced terminal output for timed runs
    const int64_t status_interval_ns = quiet ? kStatusIntervalQuietNs : kStatusIntervalNs;

    // Output: file if a name was given, otherwise the terminal.
    // Opened first so a bad path is reported before any hardware is touched.
    const bool file_mode = argc >= 2 && std::strcmp(argv[1], "-") != 0;
    std::ofstream file;
    std::ostream* out = &std::cout;
    if (file_mode) {
        file.open(argv[1], std::ios::app);
        if (!file) {
            std::fprintf(stderr, "cannot open output file %s: %s\n", argv[1], std::strerror(errno));
            return 1;
        }
        out = &file;
    }

    // Real-time setup: failures are not fatal, just less deterministic.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        std::perror("mlockall (continuing)");
    sched_param sp{};
    sp.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        std::fprintf(stderr, "SCHED_FIFO not set (run as root?) - continuing\n");

    // Signals -> file descriptor, so shutdown is just another epoll event.
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    if (pthread_sigmask(SIG_BLOCK, &mask, nullptr) != 0) {
        std::perror("pthread_sigmask");
        return 1;
    }
    int sfd = signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);
    if (sfd < 0) {
        std::perror("signalfd");
        return 1;
    }

    // Create the epoll instance and register everything on it.
    int epfd = epoll_create1(EPOLL_CLOEXEC);
    if (epfd < 0) {
        std::perror("epoll_create1");
        return 1;
    }

    // Detector i is always /dev/ttyUSB<i>, so port index == detector number.
    std::vector<Port> ports;
    ports.reserve(kNumDetectors);
    int open_ports = 0;
    for (int i = 0; i < kNumDetectors; ++i) {
        Port p;
//        p.path = "/dev/ttyUSB" + std::to_string(i);
        p.path = "dev/CW" + std::to_string(i);
        p.fd = open_serial(p.path.c_str(), speed);
        if (p.fd >= 0) {
            if (epoll_add(epfd, p.fd, uint32_t(i))) {
                ++open_ports;
            } else {
                std::perror("epoll_ctl ADD");
                ::close(p.fd);
                p.fd = -1;
            }
        }
        ports.push_back(std::move(p));       // keep slot even if failed, so indices stay stable
    }
    if (open_ports == 0) {
        std::fprintf(stderr, "no detector could be opened\n");
        ::close(sfd);
        ::close(epfd);
        return 1;
    }
    if (open_ports < kNumDetectors)
        std::fprintf(stderr, "warning: only %d of %d detectors opened\n", open_ports, kNumDetectors);
    if (!epoll_add(epfd, sfd, kSignalTag)) {
        std::perror("epoll_ctl ADD signalfd");
        return 1;
    }

    // Output lives in its own thread so write() stalls never delay reads.
    Queue queue;
    std::thread writer(writer_thread, std::ref(queue), std::ref(*out));

    // Two time references (CLOCK_MONOTONIC):
    //  t_open - ports opened; all line stamps are taken relative to it while reading,
    //  t0     - START of the readout: all opened detectors have sent header + name
    //           (or kStartTimeoutNs passed). Stored event times are relative to t0,
    //           lines that began before t0 are discarded. Wall-clock time is only printed.
    const int64_t t_open = now_ns();
    std::fprintf(stderr, "Ports opened %s, %ld baud - waiting for the detector headers (max %lld s)\n",
                 wall_stamp().c_str(), baud, (long long)(kStartTimeoutNs / kNsPerSec));

    bool running = true;
    bool started = false;
    bool none_ready = false;                 // no detector sent its header in time -> exit code 1
    int64_t t0 = 0;                          // start (absolute ns), valid once started
    int64_t t0_us = 0;                       // start relative to t_open [us]
    int64_t next_status = 0;                 // relative to t0
    epoll_event evs[kMaxEvents];
    char buf[1024];
    std::vector<Record> pending;             // events found during the current wakeup
    pending.reserve(256);
    int64_t t_end = t_open;

    while (running && open_ports > 0) {
        // Wake up at least once per second to do the periodic work,
        // earlier if the requested running time ends sooner.
        int timeout_ms = 1000;
        if (started && has_duration) {
            const int64_t left = duration_ns - (now_ns() - t0);
            if (left <= 0) break;
            timeout_ms = int(std::min<int64_t>(1000, left / 1'000'000 + 1));
        }

        int n = epoll_wait(epfd, evs, kMaxEvents, timeout_ms);
        const int64_t t = now_ns();          // stamp FIRST, before any parsing
        if (n < 0) {
            if (errno == EINTR) continue;
            std::perror("epoll_wait");
            break;
        }
        const int64_t time_us = (t - t_open) / 1000;   // line stamps: relative to t_open

        for (int i = 0; i < n; ++i) {
            const uint32_t tag = evs[i].data.u32;

            if (tag == kSignalTag) {
                signalfd_siginfo si;
                (void)!::read(sfd, &si, sizeof si);   // consume it
                running = false;
                continue;
            }

            Port& p = ports[tag];
            if (p.fd < 0) continue;

            if (evs[i].events & EPOLLIN) {
                ssize_t r = ::read(p.fd, buf, sizeof buf);
                if (r > 0) {
                    split_lines(p, buf, size_t(r), time_us, [&](const char* b, const char* e, int64_t t_line) {
                        if (e > b && e[-1] == '\r') --e;          // strip CR of CR/LF
                        if (b == e) return;                        // empty line
                        if (p.state == State::Data) {              // hot path: store as it is
                            if (started && t_line >= t0_us) {      // lines begun before the start: dropped
                                const int64_t t_rel = t_line - t0_us;
                                if (p.silent) {                    // rare: detector back after a long gap
                                    const int64_t g = t_rel - p.last_us;
                                    close_gap(p, t_rel);
                                    std::fprintf(stderr, "[%s] detector %d (%s) sends data again after %.1f s without data\n",
                                                 hms(t_rel / 1'000'000).c_str(), int(tag), p.name.c_str(), double(g) / 1e6);
                                }
                                p.last_us = t_rel;
                                ++p.events;
                                pending.push_back({uint8_t(tag), t_rel, std::string(b, e)});
                            }
                        } else {
                            handle_header_line(int(tag), p, b, e, !quiet && !started);
                            if (started && p.state == State::Data)   // late detector: name just received
                                p.last_us = (t - t0) / 1000;           // silence counted from joining
                            if (started && p.state == State::Data)
                                std::fprintf(stderr, "Detector %d (%s): %s - joined %.1f s after the start\n",
                                             int(tag), p.path.c_str(), p.name.c_str(), double(t - t0) / 1e9);
                        }
                    });
                } else if (r == 0) {
                    drop_port(epfd, p, open_ports, "EOF");
                    continue;
                } else if (errno != EAGAIN && errno != EINTR) {
                    drop_port(epfd, p, open_ports, std::strerror(errno));  // e.g. EIO on unplug
                    continue;
                }
            }
            if (evs[i].events & (EPOLLERR | EPOLLHUP))
                drop_port(epfd, p, open_ports, "device error/hangup");
        }
        queue.push_all(pending);             // one hand-over to the writer per wakeup

        // ---- start: all opened detectors ready, or timeout
        if (!started) {
            int ready = 0;
            std::string missing;
            for (int i = 0; i < kNumDetectors; ++i) {
                if (ports[i].fd < 0) continue;
                if (ports[i].state == State::Data) ++ready;
                else missing += " " + std::to_string(i);
            }
            const bool all_ready = missing.empty() && ready > 0;
            if (!all_ready && ready == 0 && t - t_open >= kStartTimeoutNs) {
                std::fprintf(stderr, "error: no detector sent its header within %lld s - no readout\n",
                             (long long)(kStartTimeoutNs / kNsPerSec));
                none_ready = true;
                running = false;
                continue;
            }
            if (all_ready || t - t_open >= kStartTimeoutNs) {
                started = true;
                t0 = t;
                t0_us = time_us;
                next_status = status_interval_ns;
                t_end = t;
                if (!missing.empty())
                    std::fprintf(stderr,
                                 "warning: detector(s)%s not ready after %lld s - readout started "
                                 "without them (they join when their header arrives)\n",
                                 missing.c_str(), (long long)(kStartTimeoutNs / kNsPerSec));
                if (quiet && ready > 0) {
                    std::string names;
                    for (int i = 0; i < kNumDetectors; ++i)
                        if (ports[i].fd >= 0 && ports[i].state == State::Data)
                            names += " " + std::to_string(i) + "=" + ports[i].name;
                    std::fprintf(stderr, "Detectors:%s\n", names.c_str());
                }
                std::fprintf(stderr, "Readout started %s, %d detector(s) ready %.1f s after opening",
                             wall_stamp().c_str(), ready, double(t - t_open) / 1e9);
                if (has_duration) std::fprintf(stderr, ", will stop after %g s", duration_s);
                std::fprintf(stderr, "\n");
            }
            continue;
        }

        // ---- periodic work
        const int64_t rel = t - t0;
        t_end = t;
        for (int i = 0; i < kNumDetectors; ++i) {          // silent-detector check (8 compares)
            Port& q = ports[i];
            if (q.fd < 0 || q.state != State::Data || q.silent) continue;
            if (rel / 1000 - q.last_us > kGapWarnNs / 1000) {
                q.silent = true;
                std::fprintf(stderr, "[%s] warning: no data from detector %d (%s) for %lld s\n",
                             hms(rel / kNsPerSec).c_str(), i, q.name.c_str(),
                             (long long)(kGapWarnNs / kNsPerSec));
            }
        }
        if (has_duration && rel >= duration_ns) {   // end: final counters follow below
            running = false;
            continue;
        }
        if (file_mode && rel >= next_status) {
            print_status(ports, rel);
            while (next_status <= rel) next_status += status_interval_ns;
        }
    }

    // ------------------------------------------------------------ shutdown
    // 1. stop watching and close every serial port
    for (Port& p : ports) drop_port(epfd, p, open_ports, "shutdown", !quiet);
    // 2. remove and close the signal fd
    epoll_ctl(epfd, EPOLL_CTL_DEL, sfd, nullptr);
    ::close(sfd);
    // 3. close the epoll instance itself last
    ::close(epfd);
    // 4. let the writer drain the queue, then join
    queue.finish();
    writer.join();

    // 5. final event counters
    if (!started) {
        if (!none_ready)
            std::fprintf(stderr, "stopped before the readout started (detectors not ready) - no data stored\n");
    } else {
        if (file_mode) print_status(ports, t_end - t0);
        // gaps without data longer than kGapWarnNs
        const int64_t end_us = (t_end - t0) / 1000;
        std::string g;
        for (int i = 0; i < kNumDetectors; ++i) {
            Port& q = ports[i];
            if (q.state != State::Data) continue;
            bool still = false;
            if (q.silent || end_us - q.last_us > kGapWarnNs / 1000) {   // ongoing at the end
                still = true;
                close_gap(q, end_us);
            }
            if (q.n_gaps == 0) continue;
            char b[256];
            std::snprintf(b, sizeof b, "%sdet %d (%s) %u gap(s), total %.0f s, longest %.0f s%s",
                          g.empty() ? "" : "; ", i, q.name.c_str(), q.n_gaps,
                          double(q.gap_total_us) / 1e6, double(q.gap_max_us) / 1e6,
                          still ? ", silent at the end" : "");
            g += b;
        }
        std::fprintf(stderr, "Gaps without data > %lld s: %s\n",
                     (long long)(kGapWarnNs / kNsPerSec), g.empty() ? "none" : g.c_str());
    }
    std::fprintf(stderr, "done\n");
    return none_ready ? 1 : 0;
}
