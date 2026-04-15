
#include <array>
#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <curl/curl.h>
#include <linux/i2c-dev.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint8_t DEFAULT_I2C_ADDR = 0x6B;
constexpr int DEFAULT_I2C_BUS = 0;
constexpr const char* DEFAULT_SERIAL_DEVICE = "/dev/ttyUSB0";
constexpr int DEFAULT_SERIAL_BAUD = 2400;

constexpr std::size_t MAX_RT_LENGTH = 64;
constexpr std::size_t MAX_DPS_LENGTH_MRDS = 80;
constexpr std::size_t MAX_DPS_LENGTH_PIRA32 = 255;
constexpr std::size_t MAX_PS_LENGTH = 8;
constexpr std::size_t MAX_AF_ITEMS_MRDS = 15;
constexpr std::size_t MAX_AF_ITEMS_PIRA32 = 25;

constexpr uint8_t REG_PI_HIGH      = 0x00;
constexpr uint8_t REG_PI_LOW       = 0x01;
constexpr uint8_t REG_PS_START     = 0x02;
constexpr uint8_t REG_PTY          = 0x0A;
constexpr uint8_t REG_DI           = 0x0B;
constexpr uint8_t REG_MS           = 0x0C;
constexpr uint8_t REG_TP           = 0x0D;
constexpr uint8_t REG_TA           = 0x0E;
constexpr uint8_t REG_AFNUM        = 0x0F;
constexpr uint8_t REG_AF_START     = 0x10;
constexpr uint8_t REG_RTEN         = 0x1F;
constexpr uint8_t REG_RT_START     = 0x20;
constexpr uint8_t REG_EXTSYNC      = 0x6E;
constexpr uint8_t REG_PHASE        = 0x6F;
constexpr uint8_t REG_STATUS       = 0x70;
constexpr uint8_t REG_SPSPER       = 0x72;
constexpr uint8_t REG_DPSMOD       = 0x73;
constexpr uint8_t REG_LABPER       = 0x74;
constexpr uint8_t REG_SCRLSPD      = 0x75;
constexpr uint8_t REG_DPSNUM       = 0x76;
constexpr uint8_t REG_DPS_START    = 0x77;

enum class EncoderType {
    Mrds1322,
    Pira32,
};

[[noreturn]] void die(const std::string& msg) {
    throw std::runtime_error(msg);
}

std::string hex8(uint8_t v) {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        << static_cast<int>(v);
    return oss.str();
}

std::string hex16(uint16_t v) {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
        << static_cast<unsigned>(v);
    return oss.str();
}

std::string trim(const std::string& s) {
    std::size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) {
        ++start;
    }
    std::size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
    }
    return s.substr(start, end - start);
}

long parseLong(const std::string& s, const std::string& what) {
    errno = 0;
    char* end = nullptr;
    long v = std::strtol(s.c_str(), &end, 0);
    if (errno != 0 || end == s.c_str() || *end != '\0') {
        die("Invalid " + what + ": " + s);
    }
    return v;
}

uint16_t parseHex16Code(std::string s, const std::string& what) {
    if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) s = s.substr(2);
    if (s.size() != 4) die(what + " must contain exactly 4 hexadecimal digits");
    for (char ch : s) {
        if (!std::isxdigit(static_cast<unsigned char>(ch))) {
            die("Invalid " + what + ": " + s);
        }
    }
    errno = 0;
    char* end = nullptr;
    unsigned long v = std::strtoul(s.c_str(), &end, 16);
    if (errno != 0 || end == s.c_str() || *end != '\0' || v > 0xFFFF) {
        die("Invalid " + what + ": " + s);
    }
    return static_cast<uint16_t>(v);
}

std::string padOrTrim(std::string s, std::size_t len) {
    if (s.size() > len) s.resize(len);
    if (s.size() < len) s.append(len - s.size(), ' ');
    return s;
}

std::vector<uint8_t> parseAfCodes(const std::string& input, std::size_t maxItems) {
    std::vector<uint8_t> out;
    std::stringstream ss(input);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (item.empty()) continue;
        if (item.find('.') != std::string::npos) {
            double mhz = std::stod(item);
            int code = static_cast<int>(std::lround((mhz - 87.5) * 10.0));
            if (code < 1 || code > 0xCC) die("AF value out of range: " + item);
            out.push_back(static_cast<uint8_t>(code));
        } else {
            long code = parseLong(item, "AF item");
            if (code < 1 || code > 0xCC) die("AF item out of range: " + item);
            out.push_back(static_cast<uint8_t>(code));
        }
    }
    if (out.size() > maxItems) die("Too many AF items");
    return out;
}

std::string afCodesToPiraString(const std::vector<uint8_t>& af) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < af.size(); ++i) {
        if (i) oss << ",";
        double mhz = 87.5 + (static_cast<int>(af[i]) / 10.0);
        oss << std::fixed << std::setprecision(1) << mhz;
    }
    return oss.str();
}

speed_t baudToTermios(int baud) {
    switch (baud) {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: die("Unsupported baud rate: " + std::to_string(baud));
    }
}

class I2CDevice {
public:
    I2CDevice(int bus, uint8_t addr) {
        std::ostringstream path;
        path << "/dev/i2c-" << bus;
        fd_ = open(path.str().c_str(), O_RDWR);
        if (fd_ < 0) die("Cannot open " + path.str() + ": " + std::strerror(errno));
        if (ioctl(fd_, I2C_SLAVE, addr) < 0) {
            int e = errno;
            close(fd_);
            fd_ = -1;
            die("Cannot select I2C address " + hex8(addr) + ": " + std::strerror(e));
        }
    }

    ~I2CDevice() {
        if (fd_ >= 0) close(fd_);
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        ssize_t n = ::write(fd_, buf, sizeof(buf));
        if (n != static_cast<ssize_t>(sizeof(buf))) {
            die("I2C write failed at reg " + hex8(reg) + ": " + std::strerror(errno));
        }
    }

    void writeBlock(uint8_t startReg, const std::vector<uint8_t>& values) {
        if (values.empty()) return;
        std::vector<uint8_t> buf;
        buf.reserve(values.size() + 1);
        buf.push_back(startReg);
        buf.insert(buf.end(), values.begin(), values.end());
        ssize_t n = ::write(fd_, buf.data(), buf.size());
        if (n != static_cast<ssize_t>(buf.size())) {
            die("I2C block write failed at reg " + hex8(startReg) + ": " + std::strerror(errno));
        }
    }

    uint8_t readRegister(uint8_t reg) {
        if (::write(fd_, &reg, 1) != 1) {
            die("I2C register select failed at reg " + hex8(reg) + ": " + std::strerror(errno));
        }
        uint8_t value = 0;
        if (::read(fd_, &value, 1) != 1) {
            die("I2C read failed at reg " + hex8(reg) + ": " + std::strerror(errno));
        }
        return value;
    }

    std::vector<uint8_t> readBlock(uint8_t startReg, std::size_t len) {
        if (::write(fd_, &startReg, 1) != 1) {
            die("I2C register select failed at reg " + hex8(startReg) + ": " + std::strerror(errno));
        }
        std::vector<uint8_t> buf(len);
        if (::read(fd_, buf.data(), len) != static_cast<ssize_t>(len)) {
            die("I2C block read failed at reg " + hex8(startReg) + ": " + std::strerror(errno));
        }
        return buf;
    }

    void sendControl(uint8_t value) {
        uint8_t b = value;
        ssize_t n = ::write(fd_, &b, 1);
        if (n != 1) {
            die("I2C control write failed: " + std::string(std::strerror(errno)));
        }
    }

private:
    int fd_{-1};
};

class SerialPort {
public:
    SerialPort(const std::string& path, int baud) {
        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) die("Cannot open serial device " + path + ": " + std::strerror(errno));

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            int e = errno;
            close(fd_);
            fd_ = -1;
            die("tcgetattr failed: " + std::string(std::strerror(e)));
        }

        cfmakeraw(&tty);
        speed_t spd = baudToTermios(baud);
        cfsetispeed(&tty, spd);
        cfsetospeed(&tty, spd);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1; // 100 ms

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            int e = errno;
            close(fd_);
            fd_ = -1;
            die("tcsetattr failed: " + std::string(std::strerror(e)));
        }

        tcflush(fd_, TCIOFLUSH);
    }

    ~SerialPort() {
        if (fd_ >= 0) close(fd_);
    }

    void writeLine(const std::string& line) {
        std::string data = line;
        data.push_back('\r');
        ssize_t n = ::write(fd_, data.data(), data.size());
        if (n != static_cast<ssize_t>(data.size())) {
            die("Serial write failed: " + std::string(std::strerror(errno)));
        }
        tcdrain(fd_);
    }

    std::string readAvailable(int totalTimeoutMs = 1200, int idleBreakMs = 150) {
        std::string out;
        char buf[256];
        int elapsed = 0;
        int idle = 0;

        while (elapsed < totalTimeoutMs) {
            ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n > 0) {
                out.append(buf, buf + n);
                idle = 0;
            } else {
                idle += 50;
                if (!out.empty() && idle >= idleBreakMs) break;
                usleep(50 * 1000);
                elapsed += 50;
            }
        }
        return out;
    }

    void flush() {
        tcflush(fd_, TCIOFLUSH);
    }

private:
    int fd_{-1};
};

struct Config {
    EncoderType encoder = EncoderType::Mrds1322;

    int bus = DEFAULT_I2C_BUS;
    uint8_t addr = DEFAULT_I2C_ADDR;

    std::string serialDevice = DEFAULT_SERIAL_DEVICE;
    int serialBaud = DEFAULT_SERIAL_BAUD;

    bool verify = true;
    bool quiet = false;
    bool hasRtUrl = false;   std::string rtUrl;
    int intervalSec = 5;
    int urlTimeoutSec = 10;

    bool hasPi = false;      uint16_t pi = 0;
    bool hasPs = false;      std::string ps;
    bool hasPty = false;     uint8_t pty = 0;
    bool hasDi = false;      uint8_t di = 0;
    bool hasMs = false;      uint8_t ms = 0;
    bool hasTp = false;      uint8_t tp = 0;
    bool hasTa = false;      uint8_t ta = 0;
    bool hasAf = false;      std::vector<uint8_t> af;
    bool clearAf = false;
    bool hasRt = false;      std::string rt;
    bool hasRtEnable = false; uint8_t rtEnable = 0;
    bool hasRtAb = false;    uint8_t rtAb = 0;
    bool hasDps = false;     std::string dps;
    bool hasDpsMode = false; uint8_t dpsMode = 0;
    bool hasLabelPeriod = false; uint8_t labelPeriod = 0;
    bool hasStaticPsPeriod = false; uint8_t staticPsPeriod = 0;
    bool hasScrollSpeed = false; uint8_t scrollSpeed = 0;
    bool clearDps = false;
    bool hasExtSync = false; uint8_t extSync = 0;
    bool hasPhase = false;   uint8_t phase = 0;
    bool wantStatus = false;
    bool rdsOn = false;
    bool rdsOff = false;
    bool reset = false;
    bool storeEeprom = false;
};

void usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " [options]\n\n"
        << "Encoder selection:\n"
        << "  --encoder TYPE      mrds1322 | pira32 (default: mrds1322)\n\n"
        << "MRDS1322 transport:\n"
        << "  -b, --bus N         I2C bus number (default: " << DEFAULT_I2C_BUS << ")\n"
        << "  -a, --addr ADDR     I2C address in hex/dec (default: 0x6B)\n\n"
        << "PIRA32 transport:\n"
        << "  --device PATH       Serial device (default: " << DEFAULT_SERIAL_DEVICE << ")\n"
        << "  --baud N            Serial speed (default: " << DEFAULT_SERIAL_BAUD << ")\n\n"
        << "Common metadata:\n"
        << "  --pi HEX            PI code, exactly 4 hex digits\n"
        << "  --ps TEXT           PS name, max 8 chars, padded with spaces\n"
        << "  --pty N             PTY 0..31\n"
        << "  --di N              DI 0..15\n"
        << "  --ms 0|1            Music/Speech\n"
        << "  --tp 0|1            Traffic Program\n"
        << "  --ta 0|1            Traffic Announcement\n"
        << "  --af LIST           AF list, comma-separated MHz or raw codes\n"
        << "  --clear-af          Clear AF list\n"
        << "  --rt TEXT           Radiotext (MRDS1322 max 64, PIRA32 RT1 max 64)\n"
        << "  --rt-url URL        Poll plain-text URL and update RT on change\n"
        << "  --interval N        Poll interval in seconds for --rt-url (default: 5)\n"
        << "  --url-timeout N     HTTP timeout in seconds for --rt-url (default: 10)\n"
        << "  --rt-enable 0|1     RT enable (MRDS1322: bit; PIRA32: RT1EN)\n"
        << "  --rt-ab 0|1         RT A/B flag (MRDS1322 only)\n"
        << "  --dps TEXT          Dynamic PS (MRDS1322 max 80, PIRA32 DPS1 up to 255)\n"
        << "  --dps-mode 0..3     DPS mode (MRDS1322: DPSMOD, PIRA32: DPS1MOD)\n"
        << "  --label-period N    Label period\n"
        << "  --static-ps-period N Static PS period\n"
        << "  --scroll-speed 0|1  Scroll speed\n"
        << "  --clear-dps         Clear Dynamic PS\n"
        << "  --extsync 0|1       External pilot synchronization\n"
        << "  --phase 0..18       Signal phase\n"
        << "  --status            Read status\n"
        << "  --rds-on            Enable RDS generator\n"
        << "  --rds-off           Disable RDS generator\n"
        << "  --reset             Reset encoder\n"
        << "  --store-eeprom      Store current settings into EEPROM\n\n"
        << "Behavior:\n"
        << "  --no-verify         Disable readback verification where possible\n"
        << "  -q, --quiet         Less verbose output\n"
        << "  -h, --help          Show this help\n\n"
        << "Examples:\n"
        << "  " << prog << " --encoder mrds1322 --bus 0 --pi C203 --ps FREEDOM --pty 10\n"
        << "  " << prog << " --encoder pira32 --device /dev/ttyUSB0 --baud 2400 --pi C203 --ps FREEDOM\n";
}

Config parseArgs(int argc, char* argv[]) {
    Config cfg;
    static const option opts[] = {
        {"encoder", required_argument, nullptr, 900},
        {"device", required_argument, nullptr, 901},
        {"baud", required_argument, nullptr, 902},
        {"bus", required_argument, nullptr, 'b'},
        {"addr", required_argument, nullptr, 'a'},
        {"pi", required_argument, nullptr, 1000},
        {"ps", required_argument, nullptr, 1001},
        {"pty", required_argument, nullptr, 1002},
        {"di", required_argument, nullptr, 1003},
        {"ms", required_argument, nullptr, 1004},
        {"tp", required_argument, nullptr, 1005},
        {"ta", required_argument, nullptr, 1006},
        {"af", required_argument, nullptr, 1007},
        {"clear-af", no_argument, nullptr, 1008},
        {"rt", required_argument, nullptr, 1009},
        {"rt-url", required_argument, nullptr, 1026},
        {"interval", required_argument, nullptr, 1027},
        {"url-timeout", required_argument, nullptr, 1028},
        {"rt-enable", required_argument, nullptr, 1010},
        {"rt-ab", required_argument, nullptr, 1011},
        {"dps", required_argument, nullptr, 1012},
        {"dps-mode", required_argument, nullptr, 1013},
        {"label-period", required_argument, nullptr, 1014},
        {"static-ps-period", required_argument, nullptr, 1015},
        {"scroll-speed", required_argument, nullptr, 1016},
        {"clear-dps", no_argument, nullptr, 1017},
        {"extsync", required_argument, nullptr, 1018},
        {"phase", required_argument, nullptr, 1019},
        {"status", no_argument, nullptr, 1020},
        {"rds-on", no_argument, nullptr, 1021},
        {"rds-off", no_argument, nullptr, 1022},
        {"reset", no_argument, nullptr, 1023},
        {"store-eeprom", no_argument, nullptr, 1024},
        {"no-verify", no_argument, nullptr, 1025},
        {"quiet", no_argument, nullptr, 'q'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int c;
    while ((c = getopt_long(argc, argv, "b:a:qh", opts, nullptr)) != -1) {
        switch (c) {
            case 900: {
                std::string v = trim(optarg);
                for (char& ch : v) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
                if (v == "mrds1322") cfg.encoder = EncoderType::Mrds1322;
                else if (v == "pira32") cfg.encoder = EncoderType::Pira32;
                else die("Unknown encoder type: " + v);
                break;
            }
            case 901: cfg.serialDevice = optarg; break;
            case 902: cfg.serialBaud = static_cast<int>(parseLong(optarg, "baud")); break;
            case 'b': {
                long v = parseLong(optarg, "bus");
                if (v < 0 || v > 255) die("Bus out of range");
                cfg.bus = static_cast<int>(v);
                break;
            }
            case 'a': {
                long v = parseLong(optarg, "address");
                if (v < 0x03 || v > 0x77) die("I2C address out of range");
                cfg.addr = static_cast<uint8_t>(v);
                break;
            }
            case 1000: cfg.pi = parseHex16Code(optarg, "PI"); cfg.hasPi = true; break;
            case 1001: cfg.ps = padOrTrim(optarg, MAX_PS_LENGTH); cfg.hasPs = true; break;
            case 1002: { long v = parseLong(optarg, "PTY"); if (v < 0 || v > 31) die("PTY must be 0..31"); cfg.pty = static_cast<uint8_t>(v); cfg.hasPty = true; break; }
            case 1003: { long v = parseLong(optarg, "DI"); if (v < 0 || v > 15) die("DI must be 0..15"); cfg.di = static_cast<uint8_t>(v); cfg.hasDi = true; break; }
            case 1004: { long v = parseLong(optarg, "MS"); if (v < 0 || v > 1) die("MS must be 0 or 1"); cfg.ms = static_cast<uint8_t>(v); cfg.hasMs = true; break; }
            case 1005: { long v = parseLong(optarg, "TP"); if (v < 0 || v > 1) die("TP must be 0 or 1"); cfg.tp = static_cast<uint8_t>(v); cfg.hasTp = true; break; }
            case 1006: { long v = parseLong(optarg, "TA"); if (v < 0 || v > 1) die("TA must be 0 or 1"); cfg.ta = static_cast<uint8_t>(v); cfg.hasTa = true; break; }
            case 1007: {
                std::size_t maxItems = (cfg.encoder == EncoderType::Pira32) ? MAX_AF_ITEMS_PIRA32 : MAX_AF_ITEMS_MRDS;
                cfg.af = parseAfCodes(optarg, maxItems);
                cfg.hasAf = true;
                break;
            }
            case 1008: cfg.clearAf = true; break;
            case 1009: cfg.rt = optarg; cfg.hasRt = true; break;
            case 1026: cfg.rtUrl = optarg; cfg.hasRtUrl = true; break;
            case 1027: { long v = parseLong(optarg, "interval"); if (v < 1 || v > 86400) die("Interval must be 1..86400"); cfg.intervalSec = static_cast<int>(v); break; }
            case 1028: { long v = parseLong(optarg, "URL timeout"); if (v < 1 || v > 300) die("URL timeout must be 1..300"); cfg.urlTimeoutSec = static_cast<int>(v); break; }
            case 1010: { long v = parseLong(optarg, "RT enable"); if (v < 0 || v > 1) die("RT enable must be 0 or 1"); cfg.rtEnable = static_cast<uint8_t>(v); cfg.hasRtEnable = true; break; }
            case 1011: { long v = parseLong(optarg, "RT A/B"); if (v < 0 || v > 1) die("RT A/B must be 0 or 1"); cfg.rtAb = static_cast<uint8_t>(v); cfg.hasRtAb = true; break; }
            case 1012: cfg.dps = optarg; cfg.hasDps = true; break;
            case 1013: { long v = parseLong(optarg, "DPS mode"); if (v < 0 || v > 3) die("DPS mode must be 0..3"); cfg.dpsMode = static_cast<uint8_t>(v); cfg.hasDpsMode = true; break; }
            case 1014: { long v = parseLong(optarg, "Label period"); if (v < 0 || v > 255) die("Label period must be 0..255"); cfg.labelPeriod = static_cast<uint8_t>(v); cfg.hasLabelPeriod = true; break; }
            case 1015: { long v = parseLong(optarg, "Static PS period"); if (v < 0 || v > 255) die("Static PS period must be 0..255"); cfg.staticPsPeriod = static_cast<uint8_t>(v); cfg.hasStaticPsPeriod = true; break; }
            case 1016: { long v = parseLong(optarg, "Scroll speed"); if (v < 0 || v > 1) die("Scroll speed must be 0 or 1"); cfg.scrollSpeed = static_cast<uint8_t>(v); cfg.hasScrollSpeed = true; break; }
            case 1017: cfg.clearDps = true; break;
            case 1018: { long v = parseLong(optarg, "EXTSYNC"); if (v < 0 || v > 1) die("EXTSYNC must be 0 or 1"); cfg.extSync = static_cast<uint8_t>(v); cfg.hasExtSync = true; break; }
            case 1019: { long v = parseLong(optarg, "PHASE"); if (v < 0 || v > 18) die("PHASE must be 0..18"); cfg.phase = static_cast<uint8_t>(v); cfg.hasPhase = true; break; }
            case 1020: cfg.wantStatus = true; break;
            case 1021: cfg.rdsOn = true; break;
            case 1022: cfg.rdsOff = true; break;
            case 1023: cfg.reset = true; break;
            case 1024: cfg.storeEeprom = true; break;
            case 1025: cfg.verify = false; break;
            case 'q': cfg.quiet = true; break;
            case 'h': usage(argv[0]); std::exit(0);
            default: usage(argv[0]); std::exit(1);
        }
    }

    if (cfg.hasRt && cfg.rt.size() > MAX_RT_LENGTH) {
        die("RT length must be <= 64");
    }
    if (cfg.hasDps) {
        std::size_t max = (cfg.encoder == EncoderType::Pira32) ? MAX_DPS_LENGTH_PIRA32 : MAX_DPS_LENGTH_MRDS;
        if (cfg.dps.size() > max) die("DPS text too long");
    }
    if (!(cfg.hasPi || cfg.hasPs || cfg.hasPty || cfg.hasDi || cfg.hasMs || cfg.hasTp || cfg.hasTa ||
          cfg.hasAf || cfg.clearAf || cfg.hasRt || cfg.hasRtUrl || cfg.hasRtEnable || cfg.hasRtAb ||
          cfg.hasDps || cfg.hasDpsMode || cfg.hasLabelPeriod || cfg.hasStaticPsPeriod ||
          cfg.hasScrollSpeed || cfg.clearDps || cfg.hasExtSync || cfg.hasPhase || cfg.wantStatus ||
          cfg.rdsOn || cfg.rdsOff || cfg.reset || cfg.storeEeprom)) {
        usage(argv[0]);
        std::exit(1);
    }

    if (cfg.encoder == EncoderType::Pira32 && cfg.hasRtAb) {
        die("--rt-ab is supported only for MRDS1322; PIRA32 handles RT A/B via RTTYPE/RT updates");
    }

    return cfg;
}


void verifyReg(I2CDevice& dev, uint8_t reg, uint8_t expected, bool verify, bool quiet);
void verifyBlock(I2CDevice& dev, uint8_t startReg, const std::vector<uint8_t>& expected, bool verify, bool quiet);

size_t curlWriteCallback(char* ptr, size_t size, size_t nmemb, void* userdata) {
    auto* out = static_cast<std::string*>(userdata);
    out->append(ptr, size * nmemb);
    return size * nmemb;
}

std::string normalizeRadiotext(std::string s) {
    for (char& ch : s) {
        unsigned char u = static_cast<unsigned char>(ch);
        if (ch == '\r' || ch == '\n' || ch == '\t') ch = ' ';
        else if (u < 32) ch = ' ';
    }
    std::string out;
    out.reserve(s.size());
    bool prevSpace = false;
    for (char ch : s) {
        if (ch == ' ') {
            if (!prevSpace) out.push_back(ch);
            prevSpace = true;
        } else {
            out.push_back(ch);
            prevSpace = false;
        }
    }
    out = trim(out);
    if (out.size() > MAX_RT_LENGTH) out.resize(MAX_RT_LENGTH);
    return out;
}

std::string fetchUrlText(const std::string& url, int timeoutSec) {
    CURL* curl = curl_easy_init();
    if (!curl) die("curl_easy_init failed");

    std::string body;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(timeoutSec));
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, static_cast<long>(timeoutSec));
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "rds_control/1.0");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);

    CURLcode rc = curl_easy_perform(curl);
    if (rc != CURLE_OK) {
        std::string msg = curl_easy_strerror(rc);
        curl_easy_cleanup(curl);
        die("HTTP fetch failed for " + url + ": " + msg);
    }

    curl_easy_cleanup(curl);
    return normalizeRadiotext(body);
}

void writeMrdsRt(I2CDevice& dev, const Config& cfg, const std::string& rtText, bool enableRt, bool* abState = nullptr) {
    if (enableRt || abState != nullptr) {
        uint8_t reg = dev.readRegister(REG_RTEN);
        if (enableRt) reg |= 0x01;
        if (abState != nullptr) {
            if (*abState) reg |= 0x02;
            else reg &= ~uint8_t{0x02};
        }
        dev.writeRegister(REG_RTEN, reg);
        verifyReg(dev, REG_RTEN, reg, cfg.verify, cfg.quiet);
    }

    std::string padded = padOrTrim(rtText, MAX_RT_LENGTH);
    std::vector<uint8_t> bytes(padded.begin(), padded.end());
    dev.writeBlock(REG_RT_START, bytes);
    verifyBlock(dev, REG_RT_START, bytes, cfg.verify, cfg.quiet);
}

void pollRtUrlMrds(I2CDevice& dev, const Config& cfg) {
    bool ab = cfg.hasRtAb ? (cfg.rtAb != 0) : false;
    std::string lastSent;

    if (!cfg.quiet) {
        std::cout << "Starting RT URL polling: " << cfg.rtUrl
                  << " (interval " << cfg.intervalSec << "s)\n";
    }

    while (true) {
        try {
            std::string text = fetchUrlText(cfg.rtUrl, cfg.urlTimeoutSec);
            if (!text.empty() && text != lastSent) {
                writeMrdsRt(dev, cfg, text, true, &ab);
                if (!cfg.quiet) {
                    std::cout << "RT update: [" << text << "] AB=" << (ab ? 1 : 0) << "\n";
                }
                lastSent = text;
                ab = !ab;
            }
        } catch (const std::exception& e) {
            std::cerr << "RT URL poll warning: " << e.what() << "\n";
        }
        sleep(cfg.intervalSec);
    }
}

void verifyReg(I2CDevice& dev, uint8_t reg, uint8_t expected, bool verify, bool quiet) {
    if (!verify) return;
    uint8_t got = dev.readRegister(reg);
    if (got != expected) {
        die("Verify failed at reg " + hex8(reg) + ": expected " + hex8(expected) + ", got " + hex8(got));
    }
    if (!quiet) {
        std::cout << "Verified " << hex8(reg) << " = " << hex8(got) << "\n";
    }
}

void verifyBlock(I2CDevice& dev, uint8_t startReg, const std::vector<uint8_t>& expected, bool verify, bool quiet) {
    if (!verify || expected.empty()) return;
    auto got = dev.readBlock(startReg, expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        if (got[i] != expected[i]) {
            die("Verify failed at reg " + hex8(static_cast<uint8_t>(startReg + i)) +
                ": expected " + hex8(expected[i]) + ", got " + hex8(got[i]));
        }
    }
    if (!quiet) {
        std::cout << "Verified block " << hex8(startReg) << ".."
                  << hex8(static_cast<uint8_t>(startReg + expected.size() - 1)) << "\n";
    }
}

void setBitfieldRtEn(I2CDevice& dev, const Config& cfg) {
    if (!(cfg.hasRtEnable || cfg.hasRtAb)) return;
    uint8_t reg = dev.readRegister(REG_RTEN);
    if (cfg.hasRtEnable) {
        if (cfg.rtEnable) reg |= 0x01; else reg &= ~uint8_t{0x01};
    }
    if (cfg.hasRtAb) {
        if (cfg.rtAb) reg |= 0x02; else reg &= ~uint8_t{0x02};
    }
    dev.writeRegister(REG_RTEN, reg);
    if (!cfg.quiet) std::cout << "Write RTEN " << hex8(reg) << "\n";
    verifyReg(dev, REG_RTEN, reg, cfg.verify, cfg.quiet);
}

void printMrdsStatus(uint8_t s) {
    std::cout << "STATUS: " << hex8(s) << "\n"
              << "  pilot_present: " << ((s & 0x01) ? "yes" : "no") << "\n"
              << "  ta_on_air:     " << ((s & 0x02) ? "yes" : "no") << "\n"
              << "  dps_running:   " << ((s & 0x04) ? "yes" : "no") << "\n"
              << "  ps_buffer_busy:" << ((s & 0x08) ? "yes" : "no") << "\n"
              << "  rds_on:        " << ((s & 0x80) ? "yes" : "no") << "\n";
}

std::vector<std::string> splitResponseLines(const std::string& raw) {
    std::vector<std::string> out;
    std::stringstream ss(raw);
    std::string line;
    while (std::getline(ss, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        line = trim(line);
        if (!line.empty()) out.push_back(line);
    }
    return out;
}

class Pira32Protocol {
public:
    Pira32Protocol(const std::string& device, int baud, bool quiet) : port_(device, baud), quiet_(quiet) {}

    std::vector<std::string> transact(const std::string& cmd, int timeoutMs = 1200, int settleMs = 0) {
        port_.flush();
        port_.writeLine(cmd);
        if (settleMs > 0) usleep(settleMs * 1000);
        auto raw = port_.readAvailable(timeoutMs, 150);
        auto lines = splitResponseLines(raw);
        // drop echoed command if present
        std::vector<std::string> filtered;
        for (const auto& line : lines) {
            if (trim(line) == trim(cmd)) continue;
            filtered.push_back(line);
        }
        return filtered;
    }

    void command(const std::string& cmd, int settleMs = 0) {
        auto lines = transact(cmd, 1200, settleMs);
        if (lines.empty()) {
            if (!quiet_) std::cout << "No explicit response for command: " << cmd << "\n";
            return;
        }
        std::string last = lines.back();
        if (last == "+" || last == "/") return;
        if (last == "!") die("PIRA32 unknown command: " + cmd);
        if (last == "-") die("PIRA32 invalid argument: " + cmd);
        // Some query-like commands may return only text; accept.
    }

    std::string queryValue(const std::string& cmd, int settleMs = 0) {
        auto lines = transact(cmd, 1500, settleMs);
        if (lines.empty()) die("No response to PIRA32 query: " + cmd);
        std::string value;
        for (const auto& line : lines) {
            if (line == "+" || line == "/" || line == "!" || line == "-") continue;
            value = line;
            break;
        }
        if (value.empty()) {
            std::string last = lines.back();
            if (last == "!") die("PIRA32 unknown command: " + cmd);
            if (last == "-") die("PIRA32 invalid argument: " + cmd);
            die("No value returned for PIRA32 query: " + cmd);
        }
        return value;
    }

    void verifyValue(const std::string& queryCmd, const std::string& expected, bool verify) {
        if (!verify) return;
        std::string got = trim(queryValue(queryCmd));
        if (trim(expected) != got) {
            die("Verify failed for " + queryCmd + ": expected [" + trim(expected) + "], got [" + got + "]");
        }
        if (!quiet_) {
            std::cout << "Verified " << queryCmd << " = [" << got << "]\n";
        }
    }

private:
    SerialPort port_;
    bool quiet_;
};


void writePiraRt(Pira32Protocol& p, const Config& cfg, const std::string& rtText, bool enableRt) {
    if (enableRt) {
        p.command("RT1EN=1");
        p.verifyValue("RT1EN", "1", cfg.verify);
    }
    p.command("RT1=" + rtText);
    p.verifyValue("RT1", rtText, cfg.verify);
}

void pollRtUrlPira(Pira32Protocol& p, const Config& cfg) {
    std::string lastSent;

    if (!cfg.quiet) {
        std::cout << "Starting RT URL polling: " << cfg.rtUrl
                  << " (interval " << cfg.intervalSec << "s)\n";
    }

    while (true) {
        try {
            std::string text = fetchUrlText(cfg.rtUrl, cfg.urlTimeoutSec);
            if (!text.empty() && text != lastSent) {
                writePiraRt(p, cfg, text, true);
                if (!cfg.quiet) {
                    std::cout << "RT update: [" << text << "]\n";
                }
                lastSent = text;
            }
        } catch (const std::exception& e) {
            std::cerr << "RT URL poll warning: " << e.what() << "\n";
        }
        sleep(cfg.intervalSec);
    }
}


void runMrds1322(const Config& cfg) {
    I2CDevice dev(cfg.bus, cfg.addr);

    if (!cfg.quiet) {
        std::cout << "MRDS1322 I2C controller\n"
                  << "Bus:     " << cfg.bus << "\n"
                  << "Address: " << hex8(cfg.addr) << "\n"
                  << "Verify:  " << (cfg.verify ? "on" : "off") << "\n\n";
    }

    if (cfg.hasPi) {
        uint8_t hi = static_cast<uint8_t>((cfg.pi >> 8) & 0xFF);
        uint8_t lo = static_cast<uint8_t>(cfg.pi & 0xFF);
        dev.writeRegister(REG_PI_HIGH, hi);
        dev.writeRegister(REG_PI_LOW, lo);
        if (!cfg.quiet) std::cout << "Write PI " << hex16(cfg.pi) << "\n";
        verifyReg(dev, REG_PI_HIGH, hi, cfg.verify, cfg.quiet);
        verifyReg(dev, REG_PI_LOW,  lo, cfg.verify, cfg.quiet);
    }

    if (cfg.hasPs) {
        std::vector<uint8_t> bytes(cfg.ps.begin(), cfg.ps.end());
        dev.writeBlock(REG_PS_START, bytes);
        if (!cfg.quiet) std::cout << "Write PS [" << cfg.ps << "]\n";
        verifyBlock(dev, REG_PS_START, bytes, cfg.verify, cfg.quiet);
    }

    if (cfg.hasPty) { dev.writeRegister(REG_PTY, cfg.pty); verifyReg(dev, REG_PTY, cfg.pty, cfg.verify, cfg.quiet); }
    if (cfg.hasDi)  { dev.writeRegister(REG_DI, cfg.di);   verifyReg(dev, REG_DI, cfg.di, cfg.verify, cfg.quiet); }
    if (cfg.hasMs)  { dev.writeRegister(REG_MS, cfg.ms);   verifyReg(dev, REG_MS, cfg.ms, cfg.verify, cfg.quiet); }
    if (cfg.hasTp)  { dev.writeRegister(REG_TP, cfg.tp);   verifyReg(dev, REG_TP, cfg.tp, cfg.verify, cfg.quiet); }
    if (cfg.hasTa)  { dev.writeRegister(REG_TA, cfg.ta);   verifyReg(dev, REG_TA, cfg.ta, cfg.verify, cfg.quiet); }

    if (cfg.clearAf) {
        dev.writeRegister(REG_AFNUM, 0);
        verifyReg(dev, REG_AFNUM, 0, cfg.verify, cfg.quiet);
        std::vector<uint8_t> zeroAf(MAX_AF_ITEMS_MRDS, 0);
        dev.writeBlock(REG_AF_START, zeroAf);
        verifyBlock(dev, REG_AF_START, zeroAf, cfg.verify, cfg.quiet);
    }
    if (cfg.hasAf) {
        dev.writeRegister(REG_AFNUM, static_cast<uint8_t>(cfg.af.size()));
        dev.writeBlock(REG_AF_START, cfg.af);
        verifyReg(dev, REG_AFNUM, static_cast<uint8_t>(cfg.af.size()), cfg.verify, cfg.quiet);
        verifyBlock(dev, REG_AF_START, cfg.af, cfg.verify, cfg.quiet);
    }

    setBitfieldRtEn(dev, cfg);

    if (cfg.hasRt) {
        writeMrdsRt(dev, cfg, cfg.rt, false);
    }

    if (cfg.clearDps) {
        dev.writeRegister(REG_DPSNUM, 0);
        verifyReg(dev, REG_DPSNUM, 0, cfg.verify, cfg.quiet);
        std::vector<uint8_t> clear(MAX_DPS_LENGTH_MRDS, ' ');
        dev.writeBlock(REG_DPS_START, clear);
        verifyBlock(dev, REG_DPS_START, clear, cfg.verify, cfg.quiet);
    }
    if (cfg.hasDpsMode) { dev.writeRegister(REG_DPSMOD, cfg.dpsMode); verifyReg(dev, REG_DPSMOD, cfg.dpsMode, cfg.verify, cfg.quiet); }
    if (cfg.hasLabelPeriod) { dev.writeRegister(REG_LABPER, cfg.labelPeriod); verifyReg(dev, REG_LABPER, cfg.labelPeriod, cfg.verify, cfg.quiet); }
    if (cfg.hasStaticPsPeriod) { dev.writeRegister(REG_SPSPER, cfg.staticPsPeriod); verifyReg(dev, REG_SPSPER, cfg.staticPsPeriod, cfg.verify, cfg.quiet); }
    if (cfg.hasScrollSpeed) { dev.writeRegister(REG_SCRLSPD, cfg.scrollSpeed); verifyReg(dev, REG_SCRLSPD, cfg.scrollSpeed, cfg.verify, cfg.quiet); }
    if (cfg.hasDps) {
        dev.writeRegister(REG_DPSNUM, 0);
        verifyReg(dev, REG_DPSNUM, 0, cfg.verify, cfg.quiet);
        std::string padded = padOrTrim(cfg.dps, MAX_DPS_LENGTH_MRDS);
        std::vector<uint8_t> bytes(padded.begin(), padded.end());
        dev.writeBlock(REG_DPS_START, bytes);
        verifyBlock(dev, REG_DPS_START, bytes, cfg.verify, cfg.quiet);
        dev.writeRegister(REG_DPSNUM, static_cast<uint8_t>(cfg.dps.size()));
        verifyReg(dev, REG_DPSNUM, static_cast<uint8_t>(cfg.dps.size()), cfg.verify, cfg.quiet);
    }

    if (cfg.hasExtSync) { dev.writeRegister(REG_EXTSYNC, cfg.extSync); verifyReg(dev, REG_EXTSYNC, cfg.extSync, cfg.verify, cfg.quiet); }
    if (cfg.hasPhase)   { dev.writeRegister(REG_PHASE, cfg.phase);     verifyReg(dev, REG_PHASE, cfg.phase, cfg.verify, cfg.quiet); }

    if (cfg.storeEeprom) {
        dev.sendControl(0x45);
        if (!cfg.quiet) std::cout << "Sent CONTROL store-to-EEPROM (0x45)\n";
    }
    if (cfg.rdsOff) {
        dev.sendControl(0x30);
        if (cfg.verify) {
            uint8_t s = dev.readRegister(REG_STATUS);
            if (s & 0x80) die("Verify failed: STATUS still reports RDS ON");
        }
    }
    if (cfg.rdsOn) {
        dev.sendControl(0x31);
        if (cfg.verify) {
            uint8_t s = dev.readRegister(REG_STATUS);
            if (!(s & 0x80)) die("Verify failed: STATUS does not report RDS ON");
        }
    }
    if (cfg.reset) {
        dev.sendControl(0x52);
    }
    if (cfg.wantStatus) {
        uint8_t s = dev.readRegister(REG_STATUS);
        printMrdsStatus(s);
    }
    if (cfg.hasRtUrl) {
        pollRtUrlMrds(dev, cfg);
    }
}

void runPira32(const Config& cfg) {
    Pira32Protocol p(cfg.serialDevice, cfg.serialBaud, cfg.quiet);

    if (!cfg.quiet) {
        std::cout << "PIRA32 serial controller\n"
                  << "Device:  " << cfg.serialDevice << "\n"
                  << "Baud:    " << cfg.serialBaud << "\n"
                  << "Verify:  " << (cfg.verify ? "on" : "off") << "\n\n";
    }

    if (cfg.hasPi) {
        std::string v = hex16(cfg.pi).substr(2);
        p.command("PI=" + v);
        p.verifyValue("PI", v, cfg.verify);
    }
    if (cfg.hasPs) {
        std::string v = cfg.ps;
        p.command("PS=" + v, 450); // PS processing may take up to ~400 ms
        p.verifyValue("PS", trim(v), cfg.verify);
    }
    if (cfg.hasPty) {
        std::string v = std::to_string(static_cast<int>(cfg.pty));
        p.command("PTY=" + v);
        p.verifyValue("PTY", v, cfg.verify);
    }
    if (cfg.hasDi) {
        std::string v = std::to_string(static_cast<int>(cfg.di));
        p.command("DI=" + v);
        p.verifyValue("DI", v, cfg.verify);
    }
    if (cfg.hasMs) {
        std::string v = std::to_string(static_cast<int>(cfg.ms));
        p.command("MS=" + v);
        p.verifyValue("MS", v, cfg.verify);
    }
    if (cfg.hasTp) {
        std::string v = std::to_string(static_cast<int>(cfg.tp));
        p.command("TP=" + v);
        p.verifyValue("TP", v, cfg.verify);
    }
    if (cfg.hasTa) {
        std::string v = std::to_string(static_cast<int>(cfg.ta));
        p.command("TA=" + v);
        p.verifyValue("TA", v, cfg.verify);
    }
    if (cfg.clearAf) {
        p.command("AF=");
        p.verifyValue("AF", "", false); // firmware formatting varies; skip strict verify
    }
    if (cfg.hasAf) {
        std::string v = afCodesToPiraString(cfg.af);
        p.command("AF=" + v);
        // AF readback formatting may vary in spacing/order formatting; skip strict verify
        if (!cfg.quiet && cfg.verify) {
            std::cout << "AF written; strict readback skipped because firmware formats AF lists textually.\n";
        }
    }
    if (cfg.hasRt) {
        writePiraRt(p, cfg, cfg.rt, false);
    }
    if (cfg.hasRtEnable) {
        std::string v = std::to_string(static_cast<int>(cfg.rtEnable));
        p.command("RT1EN=" + v);
        p.verifyValue("RT1EN", v, cfg.verify);
    }
    if (cfg.clearDps) {
        p.command("DPS1=");
        if (!cfg.quiet && cfg.verify) {
            std::cout << "DPS1 cleared; strict empty-string verify skipped.\n";
        }
    }
    if (cfg.hasDpsMode) {
        std::string v = std::to_string(static_cast<int>(cfg.dpsMode));
        p.command("DPS1MOD=" + v);
        p.verifyValue("DPS1MOD", v, cfg.verify);
    }
    if (cfg.hasLabelPeriod) {
        std::string v = std::to_string(static_cast<int>(cfg.labelPeriod));
        p.command("LABPER=" + v);
        p.verifyValue("LABPER", v, cfg.verify);
    }
    if (cfg.hasStaticPsPeriod) {
        std::string v = std::to_string(static_cast<int>(cfg.staticPsPeriod));
        p.command("SPSPER=" + v);
        p.verifyValue("SPSPER", v, cfg.verify);
    }
    if (cfg.hasScrollSpeed) {
        std::string v = std::to_string(static_cast<int>(cfg.scrollSpeed));
        p.command("SCRLSPD=" + v);
        p.verifyValue("SCRLSPD", v, cfg.verify);
    }
    if (cfg.hasDps) {
        p.command("DPS1=" + cfg.dps);
        p.verifyValue("DPS1", cfg.dps, cfg.verify);
    }
    if (cfg.hasExtSync) {
        std::string v = std::to_string(static_cast<int>(cfg.extSync));
        p.command("EXTSYNC=" + v);
        p.verifyValue("EXTSYNC", v, cfg.verify);
    }
    if (cfg.hasPhase) {
        std::string v = std::to_string(static_cast<int>(cfg.phase));
        p.command("PHASE=" + v);
        p.verifyValue("PHASE", v, cfg.verify);
    }
    if (cfg.storeEeprom) {
        p.command("*ALL");
    }
    if (cfg.rdsOff) {
        p.command("RDSGEN=0");
        p.verifyValue("RDSGEN", "0", cfg.verify);
    }
    if (cfg.rdsOn) {
        p.command("RDSGEN=1");
        p.verifyValue("RDSGEN", "1", cfg.verify);
    }
    if (cfg.reset) {
        p.command("RESET");
    }
    if (cfg.wantStatus) {
        std::string status = p.queryValue("STATUS");
        std::cout << status << "\n";
    }
    if (cfg.hasRtUrl) {
        pollRtUrlPira(p, cfg);
    }
}

} // namespace

int main(int argc, char* argv[]) {
    curl_global_init(CURL_GLOBAL_DEFAULT);
    try {
        Config cfg = parseArgs(argc, argv);
        if (cfg.encoder == EncoderType::Mrds1322) {
            runMrds1322(cfg);
        } else {
            runPira32(cfg);
        }
        if (!cfg.quiet) std::cout << "Done\n";
        curl_global_cleanup();
        return 0;
    } catch (const std::exception& e) {
        curl_global_cleanup();
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
}
