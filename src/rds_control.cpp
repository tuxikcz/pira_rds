
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
#include <linux/i2c-dev.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <csignal>
#include <curl/curl.h>
#include <ctime>

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

const std::vector<std::pair<uint8_t, const char*>> PTY_LIST = {
    {0,  "No programme type or undefined"},
    {1,  "News"},
    {2,  "Current Affairs"},
    {3,  "Information"},
    {4,  "Sport"},
    {5,  "Education"},
    {6,  "Drama"},
    {7,  "Culture"},
    {8,  "Science"},
    {9,  "Varied"},
    {10, "Pop Music"},
    {11, "Rock Music"},
    {12, "Easy Listening Music"},
    {13, "Light Classical"},
    {14, "Serious Classical"},
    {15, "Other Music"},
    {16, "Weather"},
    {17, "Finance"},
    {18, "Children's Programmes"},
    {19, "Social Affairs"},
    {20, "Religion"},
    {21, "Phone In"},
    {22, "Travel"},
    {23, "Leisure"},
    {24, "Jazz Music"},
    {25, "Country Music"},
    {26, "National Music"},
    {27, "Oldies Music"},
    {28, "Folk Music"},
    {29, "Documentary"},
    {30, "Alarm Test"},
    {31, "Alarm"},
};

volatile sig_atomic_t g_stopRequested = 0;
std::string g_pidfilePath;

void handleSignal(int) {
    g_stopRequested = 1;
}

void removePidfile() {
    if (!g_pidfilePath.empty()) {
        ::unlink(g_pidfilePath.c_str());
    }
}

void writePidfile(const std::string& path) {
    int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        throw std::runtime_error("Cannot write pidfile " + path + ": " + std::strerror(errno));
    }
    std::string pid = std::to_string(static_cast<long>(::getpid())) + "\n";
    if (::write(fd, pid.data(), pid.size()) != static_cast<ssize_t>(pid.size())) {
        int e = errno;
        ::close(fd);
        throw std::runtime_error("Cannot write pidfile " + path + ": " + std::strerror(e));
    }
    ::close(fd);
    g_pidfilePath = path;
    std::atexit(removePidfile);
}

void installSignalHandlers() {
    struct sigaction sa{};
    sa.sa_handler = handleSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);
}

void redirectStandardFds(const std::string& logfile) {
    int nullFd = ::open("/dev/null", O_RDWR);
    if (nullFd < 0) {
        throw std::runtime_error("Cannot open /dev/null: " + std::string(std::strerror(errno)));
    }
    if (::dup2(nullFd, STDIN_FILENO) < 0) {
        int e = errno;
        ::close(nullFd);
        throw std::runtime_error("dup2 stdin failed: " + std::string(std::strerror(e)));
    }

    int outFd = nullFd;
    if (!logfile.empty()) {
        outFd = ::open(logfile.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (outFd < 0) {
            int e = errno;
            ::close(nullFd);
            throw std::runtime_error("Cannot open logfile " + logfile + ": " + std::strerror(e));
        }
    }

    if (::dup2(outFd, STDOUT_FILENO) < 0 || ::dup2(outFd, STDERR_FILENO) < 0) {
        int e = errno;
        if (outFd != nullFd) ::close(outFd);
        ::close(nullFd);
        throw std::runtime_error("dup2 stdout/stderr failed: " + std::string(std::strerror(e)));
    }

    if (outFd != nullFd) ::close(outFd);
    ::close(nullFd);
}

void daemonizeProcess(const std::string& pidfile, const std::string& logfile) {
    pid_t pid = ::fork();
    if (pid < 0) {
        throw std::runtime_error("fork failed: " + std::string(std::strerror(errno)));
    }
    if (pid > 0) {
        std::exit(0);
    }

    if (::setsid() < 0) {
        throw std::runtime_error("setsid failed: " + std::string(std::strerror(errno)));
    }

    pid = ::fork();
    if (pid < 0) {
        throw std::runtime_error("second fork failed: " + std::string(std::strerror(errno)));
    }
    if (pid > 0) {
        std::exit(0);
    }

    ::umask(022);
    if (::chdir("/") != 0) {
        throw std::runtime_error("chdir(/) failed: " + std::string(std::strerror(errno)));
    }

    redirectStandardFds(logfile);
    installSignalHandlers();

    if (!pidfile.empty()) {
        writePidfile(pidfile);
    }
}

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

std::string normalizePtyName(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char ch : s) {
        unsigned char u = static_cast<unsigned char>(ch);
        if (std::isalnum(u)) out.push_back(static_cast<char>(std::tolower(u)));
        else if (std::isspace(u) || ch == '-' || ch == '_' || ch == '/' || ch == '&' || ch == '\'' ) out.push_back(' ');
    }
    std::string collapsed;
    bool lastSpace = true;
    for (char ch : out) {
        if (ch == ' ') {
            if (!lastSpace) collapsed.push_back(ch);
            lastSpace = true;
        } else {
            collapsed.push_back(ch);
            lastSpace = false;
        }
    }
    return trim(collapsed);
}

uint8_t parsePtyName(const std::string& s) {
    std::string want = normalizePtyName(s);
    if (want.empty()) die("PTY name must not be empty");
    for (const auto& item : PTY_LIST) {
        if (normalizePtyName(item.second) == want) return item.first;
    }
    die("Unknown PTY name: " + s);
}

void printPtyList() {
    std::cout << "PTY list:\n\n";
    for (const auto& item : PTY_LIST) {
        std::cout << std::setw(2) << static_cast<int>(item.first) << "\t" << item.second << "\n";
    }
    std::cout << "\n";
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
    bool daemonize = false;
    bool foreground = true;
    std::string pidfile;
    std::string logfile;
    std::string rtUrl;
    int interval = 5;
    int urlTimeout = 5;
    bool hasRtUrl = false;

    bool hasPi = false;      uint16_t pi = 0;
    bool hasPs = false;      std::string ps;
    bool hasPty = false;     uint8_t pty = 0;
    bool wantPtyList = false;
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
        << "  --pty N|list        PTY 0..31 or print PTY list\n"
        << "  --pty-name NAME     Set PTY by name, e.g. \"Rock Music\"\n"
        << "  --list              Print PTY list and exit\n"
        << "  --di N              DI 0..15\n"
        << "  --ms 0|1            Music/Speech\n"
        << "  --tp 0|1            Traffic Program\n"
        << "  --ta 0|1            Traffic Announcement\n"
        << "  --af LIST           AF list, comma-separated MHz or raw codes\n"
        << "  --clear-af          Clear AF list\n"
        << "  --rt TEXT           Radiotext (MRDS1322 max 64, PIRA32 RT1 max 64)\n"
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
        << "  --daemon            Run in background\n"
        << "  --foreground        Stay in foreground (default)\n"
        << "  --pidfile FILE      Write daemon PID to file\n"
        << "  --logfile FILE      Redirect stdout/stderr to logfile in daemon mode\n"
        << "  --rt-url URL        Poll plain-text URL and send changes as RT\n"
        << "  --interval N        Poll interval in seconds for --rt-url (default: 5)\n"
        << "  --url-timeout N     HTTP timeout in seconds for --rt-url (default: 5)\n"
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
        {"pty-name", required_argument, nullptr, 1029},
        {"list", no_argument, nullptr, 1030},
        {"di", required_argument, nullptr, 1003},
        {"ms", required_argument, nullptr, 1004},
        {"tp", required_argument, nullptr, 1005},
        {"ta", required_argument, nullptr, 1006},
        {"af", required_argument, nullptr, 1007},
        {"clear-af", no_argument, nullptr, 1008},
        {"rt", required_argument, nullptr, 1009},
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
        {"daemon", no_argument, nullptr, 1026},
        {"foreground", no_argument, nullptr, 1027},
        {"pidfile", required_argument, nullptr, 1028},
        {"rt-url", required_argument, nullptr, 1032},
        {"interval", required_argument, nullptr, 1033},
        {"url-timeout", required_argument, nullptr, 1034},
        {"logfile", required_argument, nullptr, 1031},
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
            case 1002: {
                std::string v = trim(optarg);
                std::string vl = v;
                for (char& ch : vl) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
                if (vl == "list") {
                    cfg.wantPtyList = true;
                    break;
                }
                long n = parseLong(v, "PTY");
                if (n < 0 || n > 31) die("PTY must be 0..31");
                cfg.pty = static_cast<uint8_t>(n);
                cfg.hasPty = true;
                break;
            }
            case 1029:
                cfg.pty = parsePtyName(optarg);
                cfg.hasPty = true;
                break;
            case 1030:
                cfg.wantPtyList = true;
                break;
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
            case 1026: cfg.daemonize = true; cfg.foreground = false; break;
            case 1027: cfg.foreground = true; cfg.daemonize = false; break;
            case 1028: cfg.pidfile = optarg; break;
            case 1032: cfg.rtUrl = optarg; cfg.hasRtUrl = true; break;
            case 1033: { long v = parseLong(optarg, "interval"); if (v < 1 || v > 86400) die("interval must be 1..86400"); cfg.interval = static_cast<int>(v); break; }
            case 1034: { long v = parseLong(optarg, "url-timeout"); if (v < 1 || v > 300) die("url-timeout must be 1..300"); cfg.urlTimeout = static_cast<int>(v); break; }
            case 1031: cfg.logfile = optarg; break;
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
    if (!(cfg.wantPtyList || cfg.hasPi || cfg.hasPs || cfg.hasPty || cfg.hasDi || cfg.hasMs || cfg.hasTp || cfg.hasTa ||
          cfg.hasAf || cfg.clearAf || cfg.hasRt || cfg.hasRtEnable || cfg.hasRtAb ||
          cfg.hasDps || cfg.hasDpsMode || cfg.hasLabelPeriod || cfg.hasStaticPsPeriod ||
          cfg.hasScrollSpeed || cfg.clearDps || cfg.hasExtSync || cfg.hasPhase || cfg.wantStatus ||
          cfg.rdsOn || cfg.rdsOff || cfg.reset || cfg.storeEeprom || cfg.hasRtUrl)) {
        usage(argv[0]);
        std::exit(1);
    }

    if (cfg.encoder == EncoderType::Pira32 && cfg.hasRtAb) {
        die("--rt-ab is supported only for MRDS1322; PIRA32 handles RT A/B via RTTYPE/RT updates");
    }

    return cfg;
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


size_t curlWriteCallback(char* ptr, size_t size, size_t nmemb, void* userdata) {
    auto* out = static_cast<std::string*>(userdata);
    out->append(ptr, size * nmemb);
    return size * nmemb;
}

std::string decodeHtmlEntities(std::string s) {
    const std::array<std::pair<const char*, const char*>, 12> named = {{
        {"&amp;", "&"},
        {"&lt;", "<"},
        {"&gt;", ">"},
        {"&quot;", "\""},
        {"&apos;", "'"},
        {"&#039;", "'"},
        {"&#39;", "'"},
        {"&nbsp;", " "},
        {"&ndash;", "-"},
        {"&mdash;", "-"},
        {"&hellip;", "..."},
        {"&rsquo;", "'"}
    }};

    for (const auto& item : named) {
        std::size_t pos = 0;
        while ((pos = s.find(item.first, pos)) != std::string::npos) {
            s.replace(pos, std::strlen(item.first), item.second);
            pos += std::strlen(item.second);
        }
    }

    std::string out;
    out.reserve(s.size());
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '&' && i + 3 < s.size() && s[i + 1] == '#') {
            std::size_t j = i + 2;
            int base = 10;
            if (j < s.size() && (s[j] == 'x' || s[j] == 'X')) {
                base = 16;
                ++j;
            }
            std::size_t numStart = j;
            while (j < s.size() && std::isxdigit(static_cast<unsigned char>(s[j]))) ++j;
            if (j < s.size() && s[j] == ';' && j > numStart) {
                unsigned long code = 0;
                try {
                    code = std::stoul(s.substr(numStart, j - numStart), nullptr, base);
                } catch (...) {
                    code = 0;
                }
                if (code >= 32 && code <= 126) {
                    out.push_back(static_cast<char>(code));
                } else if (code == 160) {
                    out.push_back(' ');
                } else {
                    out.push_back(' ');
                }
                i = j;
                continue;
            }
        }
        out.push_back(s[i]);
    }
    return out;
}

std::string normalizeRtText(std::string s) {
    s = decodeHtmlEntities(std::move(s));
    for (char& ch : s) {
        if (ch == '\r' || ch == '\n' || ch == '\t') ch = ' ';
    }
    std::string out;
    out.reserve(s.size());
    bool prevSpace = false;
    for (unsigned char ch : s) {
        bool isSpace = std::isspace(ch);
        if (isSpace) {
            if (!prevSpace) out.push_back(' ');
        } else {
            out.push_back(static_cast<char>(ch));
        }
        prevSpace = isSpace;
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
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, timeoutSec);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeoutSec);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "rds_control/1.0");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
    CURLcode rc = curl_easy_perform(curl);
    long code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
    curl_easy_cleanup(curl);
    if (rc != CURLE_OK) {
        die(std::string("HTTP fetch failed: ") + curl_easy_strerror(rc));
    }
    if (code < 200 || code >= 300) {
        die("HTTP fetch failed with status " + std::to_string(code));
    }
    return normalizeRtText(body);
}

void sleepInterruptible(int seconds) {
    for (int i = 0; i < seconds && !g_stopRequested; ++i) {
        ::sleep(1);
    }
}

void writeMrdsRtText(I2CDevice& dev, const std::string& rt, bool verify, bool quiet) {
    uint8_t rten = dev.readRegister(REG_RTEN);
    rten |= 0x01;
    rten ^= 0x02; // toggle A/B on each change
    dev.writeRegister(REG_RTEN, rten);
    verifyReg(dev, REG_RTEN, rten, verify, quiet);

    std::string padded = padOrTrim(rt, MAX_RT_LENGTH);
    std::vector<uint8_t> bytes(padded.begin(), padded.end());
    dev.writeBlock(REG_RT_START, bytes);
    verifyBlock(dev, REG_RT_START, bytes, verify, quiet);
}

void writePira32RtText(Pira32Protocol& p, const std::string& rt, bool verify) {
    p.command("RT1EN=1");
    p.verifyValue("RT1EN", "1", verify);
    p.command("RT1=" + rt);
    p.verifyValue("RT1", rt, verify);
}

void runRtUrlLoop(const Config& cfg) {
    if (!cfg.quiet) {
        std::cout << "RT URL mode\n"
                  << "URL:      " << cfg.rtUrl << "\n"
                  << "Interval: " << cfg.interval << " s\n"
                  << "Timeout:  " << cfg.urlTimeout << " s\n\n";
    }

    std::string lastRt;
    bool first = true;

    if (cfg.encoder == EncoderType::Mrds1322) {
        I2CDevice dev(cfg.bus, cfg.addr);
        if (cfg.rdsOn) {
            dev.sendControl(0x31);
        }
        while (!g_stopRequested) {
            try {
                std::string rt = fetchUrlText(cfg.rtUrl, cfg.urlTimeout);
                if (!rt.empty() && (first || rt != lastRt)) {
                    writeMrdsRtText(dev, rt, cfg.verify, cfg.quiet);
                    if (!cfg.quiet) {
                        std::cout << "[" << std::time(nullptr) << "] RT update: " << rt << "\n";
                    }
                    lastRt = rt;
                    first = false;
                }
            } catch (const std::exception& e) {
                if (!cfg.quiet) {
                    std::cerr << "RT fetch/update error: " << e.what() << "\n";
                }
            }
            sleepInterruptible(cfg.interval);
        }
    } else {
        Pira32Protocol p(cfg.serialDevice, cfg.serialBaud, cfg.quiet);
        if (cfg.rdsOn) {
            p.command("RDSGEN=1");
            p.verifyValue("RDSGEN", "1", cfg.verify);
        }
        while (!g_stopRequested) {
            try {
                std::string rt = fetchUrlText(cfg.rtUrl, cfg.urlTimeout);
                if (!rt.empty() && (first || rt != lastRt)) {
                    writePira32RtText(p, rt, cfg.verify);
                    if (!cfg.quiet) {
                        std::cout << "[" << std::time(nullptr) << "] RT update: " << rt << "\n";
                    }
                    lastRt = rt;
                    first = false;
                }
            } catch (const std::exception& e) {
                if (!cfg.quiet) {
                    std::cerr << "RT fetch/update error: " << e.what() << "\n";
                }
            }
            sleepInterruptible(cfg.interval);
        }
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
        std::string padded = padOrTrim(cfg.rt, MAX_RT_LENGTH);
        std::vector<uint8_t> bytes(padded.begin(), padded.end());
        dev.writeBlock(REG_RT_START, bytes);
        verifyBlock(dev, REG_RT_START, bytes, cfg.verify, cfg.quiet);
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
        p.command("RT1=" + cfg.rt);
        p.verifyValue("RT1", cfg.rt, cfg.verify);
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
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        Config cfg = parseArgs(argc, argv);
        if (cfg.daemonize) {
            daemonizeProcess(cfg.pidfile, cfg.logfile);
        } else {
            installSignalHandlers();
        }
        if (cfg.wantPtyList) {
            printPtyList();
            return 0;
        }
        if (cfg.hasRtUrl) {
            runRtUrlLoop(cfg);
        } else if (cfg.encoder == EncoderType::Mrds1322) {
            runMrds1322(cfg);
        } else {
            runPira32(cfg);
        }
        if (g_stopRequested && !cfg.quiet) std::cout << "Stopped by signal\n";
        else if (!cfg.quiet) std::cout << "Done\n";
        curl_global_cleanup();
        return 0;
    } catch (const std::exception& e) {
        curl_global_cleanup();
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
}
