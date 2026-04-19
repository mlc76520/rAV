/*
Dual SSD1309 OLED Audio Visualizer for Raspberry Pi 3B / 5
Unified C++ implementation with hardware abstraction layer

=== RPi 5 (lgpio) ===
sudo apt install g++ pkg-config libdbus-1-dev libasound2-dev libfftw3-dev libfreetype-dev libmpdclient-dev liblgpio-dev

g++ -o visualizer visualizer.cpp \
    -O3 -march=native -std=c++23 \
    -I/usr/include/freetype2 \
    $(pkg-config --cflags --libs dbus-1) \
    -fstack-protector-strong -D_FORTIFY_SOURCE=2 -fPIE -pie -Wl,-z,relro,-z,now \
    -llgpio -lpthread -lasound -lfftw3f -lm -lfreetype -lmpdclient

=== RPi 3B (bcm2835) ===
sudo apt install g++ pkg-config libdbus-1-dev libasound2-dev libfftw3-dev libfreetype-dev libmpdclient-dev
# Install bcm2835 library from https://www.airspayce.com/mikem/bcm2835/

g++ -o visualizer visualizer.cpp \
    -O3 -march=native -std=c++17 -DUSE_BCM2835 \
    -I/usr/include/freetype2 \
    $(pkg-config --cflags --libs dbus-1) \
    -fstack-protector-strong -D_FORTIFY_SOURCE=2 -fPIE -pie -Wl,-z,relro,-z,now \
    -lbcm2835 -lpthread -lasound -lfftw3f -lm -lfreetype -lmpdclient
*/

#ifdef USE_BCM2835
#include <bcm2835.h>
#else
#include <lgpio.h>
#endif

#include <alsa/asoundlib.h>
#include <fftw3.h>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <thread>
#include <atomic>
#include <chrono>
#include <array>
#include <mutex>
#include <signal.h>
#include <ft2build.h>
#include <mpd/client.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <memory>
#include <dbus/dbus.h>
#include <functional>
#include <unistd.h>
#include FT_FREETYPE_H

// --- Configuration Constants ---
namespace Config {
    constexpr int SCREEN_WIDTH = 128;
    constexpr int SCREEN_HEIGHT = 64;
    constexpr int SPI_CHUNK_SIZE = 1024;
    constexpr uint8_t LEFT_CS = 8;
    constexpr uint8_t LEFT_DC = 25;
    constexpr uint8_t LEFT_RST = 24;
    constexpr uint8_t RIGHT_CS = 7;
    constexpr uint8_t RIGHT_DC = 23;
    constexpr uint8_t RIGHT_RST = 22;
    constexpr uint8_t VIZ_SW = 27;
    constexpr uint8_t RW_SW = 6;
    constexpr uint8_t FW_SW = 9;
    constexpr uint8_t PLAY_SW = 26;
    constexpr uint8_t POWER_LED = 16;
    constexpr uint8_t ROT_CLK = 17;
    constexpr uint8_t ROT_DT = 5;
    constexpr uint8_t ROT_SW = 13;
    constexpr int GPIO_CHIP = 4;
    constexpr int SPI_DEVICE = 0;
    constexpr int SPI_CHANNEL = 0;
    constexpr int SPI_SPEED = 10000000;
}

// ============================================================
// Hardware Abstraction Layer
// ============================================================

class HardwareLayer {
public:
    virtual ~HardwareLayer() = default;
    virtual bool init() = 0;
    virtual void cleanup() = 0;

    // GPIO
    virtual int claimOutput(int pin, int initialValue = 0) = 0;
    virtual int claimInput(int pin) = 0;
    virtual int claimInputPullUp(int pin) = 0;
    virtual void digitalWrite(int pin, int value) = 0;
    virtual int digitalRead(int pin) = 0;

    // SPI
    virtual void spiTransfer(int cs_pin, uint8_t data) = 0;
    virtual void spiWrite(int cs_pin, const char* buf, int len) = 0;

    // Timing
    virtual void delay(int ms) = 0;
    virtual void delayMicroseconds(int us) = 0;

    // Rotary encoder support (interrupt-based on lgpio, polling on bcm2835)
    virtual bool supportsAlerts() const = 0;

    // lgpio-specific alert functions (no-op on bcm2835)
    virtual int gpioHandle() const = 0;
    virtual int claimAlert(int pin, int flags, int edge) { return -1; }
    virtual int setDebounce(int pin, int us) { return -1; }
    virtual int setAlertsFunc(int pin, void* func, void* user) { return -1; }
    virtual int clearAlertsFunc(int pin) { return -1; }
};

// --- lgpio backend (RPi5) ---
#ifndef USE_BCM2835

class LGpioBackend : public HardwareLayer {
    int gpio_handle_ = -1;
    int spi_cs0_ = -1;
    int spi_cs1_ = -1;

public:
    bool init() override {
        gpio_handle_ = lgGpiochipOpen(Config::GPIO_CHIP);
        if (gpio_handle_ < 0) {
            gpio_handle_ = lgGpiochipOpen(0);
            if (gpio_handle_ < 0) {
                printf("Failed to open GPIO chip: %s\n", lguErrorText(gpio_handle_));
                return false;
            }
            printf("Using gpiochip0 (fallback)\n");
        } else {
            printf("Using gpiochip4 (RPi5)\n");
        }

        spi_cs0_ = lgSpiOpen(Config::SPI_DEVICE, 0, Config::SPI_SPEED, 0);
        if (spi_cs0_ < 0) {
            printf("Failed to open SPI CS0: %s\n", lguErrorText(spi_cs0_));
            lgGpiochipClose(gpio_handle_); gpio_handle_ = -1;
            return false;
        }

        spi_cs1_ = lgSpiOpen(Config::SPI_DEVICE, 1, Config::SPI_SPEED, 0);
        if (spi_cs1_ < 0) {
            printf("Failed to open SPI CS1: %s\n", lguErrorText(spi_cs1_));
            lgSpiClose(spi_cs0_); lgGpiochipClose(gpio_handle_);
            gpio_handle_ = -1; spi_cs0_ = -1;
            return false;
        }

        printf("SPI initialized: CS0 and CS1 at %d Hz\n", Config::SPI_SPEED);
        return true;
    }

    void cleanup() override {
        if (spi_cs0_ >= 0) { lgSpiClose(spi_cs0_); spi_cs0_ = -1; }
        if (spi_cs1_ >= 0) { lgSpiClose(spi_cs1_); spi_cs1_ = -1; }
        if (gpio_handle_ >= 0) { lgGpiochipClose(gpio_handle_); gpio_handle_ = -1; }
    }

    int claimOutput(int pin, int initialValue = 0) override {
        return lgGpioClaimOutput(gpio_handle_, 0, pin, initialValue);
    }
    int claimInput(int pin) override {
        return lgGpioClaimInput(gpio_handle_, 0, pin);
    }
    int claimInputPullUp(int pin) override {
        return lgGpioClaimInput(gpio_handle_, LG_SET_PULL_UP, pin);
    }
    void digitalWrite(int pin, int value) override {
        lgGpioWrite(gpio_handle_, pin, value);
    }
    int digitalRead(int pin) override {
        return lgGpioRead(gpio_handle_, pin);
    }

    void spiTransfer(int cs_pin, uint8_t data) override {
        char tx = data, rx = 0;
        int h = (cs_pin == Config::LEFT_CS) ? spi_cs0_ : spi_cs1_;
        lgSpiXfer(h, &tx, &rx, 1);
    }
    void spiWrite(int cs_pin, const char* buf, int len) override {
        int h = (cs_pin == Config::LEFT_CS) ? spi_cs0_ : spi_cs1_;
        lgSpiWrite(h, buf, len);
    }

    void delay(int ms) override { lguSleep(ms / 1000.0); }
    void delayMicroseconds(int us) override { lguSleep(us / 1000000.0); }

    bool supportsAlerts() const override { return true; }
    int gpioHandle() const override { return gpio_handle_; }

    int claimAlert(int pin, int flags, int edge) override {
        return lgGpioClaimAlert(gpio_handle_, flags, edge, pin, -1);
    }
    int setDebounce(int pin, int us) override {
        return lgGpioSetDebounce(gpio_handle_, pin, us);
    }
    int setAlertsFunc(int pin, void* func, void* user) override {
        return lgGpioSetAlertsFunc(gpio_handle_, pin, (lgGpioAlertsFunc_t)func, user);
    }
    int clearAlertsFunc(int pin) override {
        return lgGpioSetAlertsFunc(gpio_handle_, pin, nullptr, nullptr);
    }
};

#else // USE_BCM2835

// --- bcm2835 backend (RPi3B) ---
class BCM2835Backend : public HardwareLayer {
public:
    bool init() override {
        if (!bcm2835_init()) {
            printf("bcm2835_init failed\n");
            return false;
        }
        if (!bcm2835_spi_begin()) {
            printf("bcm2835_spi_begin failed\n");
            bcm2835_close();
            return false;
        }
        bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
        bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
        bcm2835_spi_setClockDivider(40); // ~6.25MHz (250MHz/40) stable without capacitor

        printf("bcm2835 SPI initialized\n");
        return true;
    }

    void cleanup() override {
        bcm2835_spi_end();
        bcm2835_close();
    }

    int claimOutput(int pin, int initialValue = 0) override {
        bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_OUTP);
        bcm2835_gpio_write(pin, initialValue);
        return 0;
    }
    int claimInput(int pin) override {
        bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_INPT);
        return 0;
    }
    int claimInputPullUp(int pin) override {
        bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_set_pud(pin, BCM2835_GPIO_PUD_UP);
        return 0;
    }
    void digitalWrite(int pin, int value) override {
        bcm2835_gpio_write(pin, value);
    }
    int digitalRead(int pin) override {
        return bcm2835_gpio_lev(pin);
    }

    void spiTransfer(int cs_pin, uint8_t data) override {
        bcm2835_gpio_write(cs_pin, 0);
        bcm2835_spi_transfer(data);
        bcm2835_gpio_write(cs_pin, 1);
    }
    void spiWrite(int cs_pin, const char* buf, int len) override {
        bcm2835_gpio_write(cs_pin, 0);
        bcm2835_spi_transfern((char*)buf, len);
        bcm2835_gpio_write(cs_pin, 1);
    }

    void delay(int ms) override { 
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
    void delayMicroseconds(int us) override { 
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }

    bool supportsAlerts() const override { return false; }
    int gpioHandle() const override { return -1; }
};

#endif // USE_BCM2835

// Global hardware pointer
static std::unique_ptr<HardwareLayer> hw;

// Auto-detect and create the right backend
std::unique_ptr<HardwareLayer> createHardware() {
#ifdef USE_BCM2835
    return std::make_unique<BCM2835Backend>();
#else
    return std::make_unique<LGpioBackend>();
#endif
}

// Forward declarations
class Display;
class AudioProcessor;
struct ControlState;

// --- MPD Client ---
class MPDClient {
    private:
        struct mpd_connection* conn;
        std::thread mpd_thread;
        std::atomic<bool> thread_running;
        std::mutex data_mutex;

        std::string formatted_text;
        std::string host;
        int port;

        std::string track_number;
        std::string title;
        std::string artist;
        std::string album;
        std::string date;
        std::string bitrate_str;
        std::string format_str;
        unsigned int elapsed_time;
        unsigned int total_time;
        bool is_playing;

        std::chrono::steady_clock::time_point last_status_update;
        std::chrono::steady_clock::time_point last_bitrate_check;

        void updateFormattedText() {
            std::lock_guard<std::mutex> lock(data_mutex);

            if (!conn || mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) {
                formatted_text = "Waiting for MPD...";
                track_number = ""; title = "No song playing"; artist = ""; album = "";
                date = ""; bitrate_str = ""; format_str = "";
                elapsed_time = 0; total_time = 0; is_playing = false;
                return;
            }

            struct mpd_song* song = mpd_run_current_song(conn);
            if (!song) {
                formatted_text = "No song playing";
                track_number = ""; title = "No song playing"; artist = ""; album = "";
                date = ""; bitrate_str = ""; format_str = "";
                elapsed_time = 0; total_time = 0; is_playing = false;
                mpd_response_finish(conn);
                return;
            }

            unsigned track = mpd_song_get_pos(song) + 1;
            const char* title_tag = mpd_song_get_tag(song, MPD_TAG_TITLE, 0);
            const char* artist_tag = mpd_song_get_tag(song, MPD_TAG_ARTIST, 0);
            const char* album_tag = mpd_song_get_tag(song, MPD_TAG_ALBUM, 0);
            const char* date_tag = mpd_song_get_tag(song, MPD_TAG_DATE, 0);

            track_number = std::to_string(track);
            title = title_tag ? title_tag : "Unknown Title";
            artist = artist_tag ? artist_tag : "Unknown Artist";
            album = album_tag ? album_tag : "Unknown Album";
            date = (date_tag && strlen(date_tag) >= 4) ? std::string(date_tag).substr(0, 4) : "";

            std::stringstream ss;
            ss << std::setfill('0') << std::setw(2) << track << ". ";
            ss << (title_tag ? title_tag : "Unknown Title");
            if (artist_tag) ss << " - " << artist_tag;
            if (date_tag && strlen(date_tag) >= 4) ss << " (" << std::string(date_tag).substr(0, 4) << ")";

            formatted_text = ss.str();
            mpd_song_free(song);
            mpd_response_finish(conn);

            struct mpd_status* status = mpd_run_status(conn);
            if (status) {
                elapsed_time = mpd_status_get_elapsed_time(status);
                total_time = mpd_status_get_total_time(status);
                is_playing = (mpd_status_get_state(status) == MPD_STATE_PLAY);

                unsigned int kbps = mpd_status_get_kbit_rate(status);
                bitrate_str = (kbps > 0) ? (std::to_string(kbps) + " kbps") : "N/A";

                const struct mpd_audio_format* format = mpd_status_get_audio_format(status);
                if (format) {
                    std::stringstream fmt;
                    fmt << (format->sample_rate / 1000) << "kHz/"
                        << (int)format->bits << "bit/"
                        << (int)format->channels << "ch";
                    format_str = fmt.str();
                } else {
                    format_str = "N/A";
                }

                mpd_status_free(status);
                last_status_update = std::chrono::steady_clock::now();
            } else {
                bitrate_str = "N/A"; format_str = "N/A"; is_playing = false;
            }
            mpd_response_finish(conn);
        }

        void refreshBitrateOnly() {
            if (!conn || mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) return;

            struct mpd_status* status = mpd_run_status(conn);
            if (status) {
                std::lock_guard<std::mutex> lock(data_mutex);
                elapsed_time = mpd_status_get_elapsed_time(status);
                is_playing = (mpd_status_get_state(status) == MPD_STATE_PLAY);
                unsigned int kbps = mpd_status_get_kbit_rate(status);
                bitrate_str = (kbps > 0) ? (std::to_string(kbps) + " kbps") : "N/A";
                mpd_status_free(status);
                last_bitrate_check = std::chrono::steady_clock::now();
            }
            mpd_response_finish(conn);
        }

        void mpdThreadFunc() {
            printf("MPD thread started\n");

            while (thread_running) {
                if (!conn) {
                    conn = mpd_connection_new(host.c_str(), port, 5000);
                    if (!conn || mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) {
                        if (conn) { mpd_connection_free(conn); conn = nullptr; }
                        for (int i = 0; i < 50; ++i) {
                            if (!thread_running) return;
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                        continue;
                    }
                    printf("Connected to MPD at %s:%d\n", host.c_str(), port);
                    updateFormattedText();
                }

                if (!mpd_send_idle_mask(conn, MPD_IDLE_PLAYER)) {
                    printf("MPD idle failed, reconnecting...\n");
                    mpd_connection_free(conn); conn = nullptr;
                    continue;
                }

                int fd = mpd_connection_get_fd(conn);
                if (fd < 0) {
                    printf("Invalid MPD connection fd\n");
                    mpd_connection_free(conn); conn = nullptr;
                    continue;
                }

                fd_set read_fds;
                struct timeval tv;

                while (thread_running) {
                    FD_ZERO(&read_fds);
                    FD_SET(fd, &read_fds);
                    tv.tv_sec = 0; tv.tv_usec = 100000;

                    int ret = select(fd + 1, &read_fds, nullptr, nullptr, &tv);

                    if (ret < 0) {
                        printf("Select error, reconnecting...\n");
                        mpd_connection_free(conn); conn = nullptr;
                        break;
                    } else if (ret == 0) {
                        auto now = std::chrono::steady_clock::now();
                        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - last_bitrate_check).count();

                        if (diff >= 1000) {
                            mpd_run_noidle(conn);
                            if (mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) {
                                printf("MPD connection error after noidle: %s\n",
                                    mpd_connection_get_error_message(conn));
                                mpd_connection_free(conn); conn = nullptr;
                                break;
                            }
                            refreshBitrateOnly();
                            break;
                        }
                        continue;
                    } else {
                        enum mpd_idle events = mpd_recv_idle(conn, false);
                        if (mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) {
                            printf("MPD connection error: %s\n",
                                mpd_connection_get_error_message(conn));
                            mpd_connection_free(conn); conn = nullptr;
                            break;
                        }
                        if (events & MPD_IDLE_PLAYER) updateFormattedText();
                        break;
                    }
                }
            }

            if (conn) { mpd_connection_free(conn); conn = nullptr; }
            printf("MPD thread stopped\n");
        }

        void withMPD(std::function<void(mpd_connection*)> action) {
            struct mpd_connection* ctrl = mpd_connection_new(host.c_str(), port, 1000);
            if (ctrl && mpd_connection_get_error(ctrl) == MPD_ERROR_SUCCESS)
                action(ctrl);
            if (ctrl) mpd_connection_free(ctrl);
        }

    public:
        MPDClient(const std::string& mpd_host = "localhost", int mpd_port = 6600)
            : conn(nullptr), thread_running(false), host(mpd_host), port(mpd_port) {
            formatted_text = "Waiting for MPD...";
            track_number = ""; title = "No song playing"; artist = ""; album = "";
            date = ""; bitrate_str = ""; format_str = "";
            elapsed_time = 0; total_time = 0; is_playing = false;
            last_status_update = std::chrono::steady_clock::now();
            last_bitrate_check = std::chrono::steady_clock::now();
        }

        ~MPDClient() { stop(); }

        bool start() {
            if (thread_running) return true;
            thread_running = true;
            mpd_thread = std::thread(&MPDClient::mpdThreadFunc, this);
            return true;
        }

        void stop() {
            if (!thread_running) return;
            thread_running = false;
            if (mpd_thread.joinable()) mpd_thread.join();
        }

        std::string getFormattedText() { std::lock_guard<std::mutex> lock(data_mutex); return formatted_text; }
        std::string getTrackNumber()   { std::lock_guard<std::mutex> lock(data_mutex); return track_number; }
        std::string getTitle()         { std::lock_guard<std::mutex> lock(data_mutex); return title; }
        std::string getArtist()        { std::lock_guard<std::mutex> lock(data_mutex); return artist; }
        std::string getAlbum()         { std::lock_guard<std::mutex> lock(data_mutex); return album; }
        std::string getDate()          { std::lock_guard<std::mutex> lock(data_mutex); return date; }
        std::string getBitrate()       { std::lock_guard<std::mutex> lock(data_mutex); return bitrate_str; }
        std::string getFormat()        { std::lock_guard<std::mutex> lock(data_mutex); return format_str; }
        unsigned int getElapsedTime()  { std::lock_guard<std::mutex> lock(data_mutex); return elapsed_time; }
        unsigned int getTotalTime()    { std::lock_guard<std::mutex> lock(data_mutex); return total_time; }

        std::string formatTime(unsigned int seconds) {
            char buffer[16];
            snprintf(buffer, sizeof(buffer), "%02u:%02u", seconds / 60, seconds % 60);
            return std::string(buffer);
        }

        void playPause() {
            withMPD([this](auto* c) {
                if (is_playing) mpd_run_toggle_pause(c);
                else mpd_run_play(c);
            });
        }
        void nextTrack() { withMPD([](auto* c) { mpd_run_next(c); }); }
        void prevTrack() { withMPD([](auto* c) { mpd_run_previous(c); }); }

        void updateElapsedTime() {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (is_playing && total_time > 0) {
                auto now = std::chrono::steady_clock::now();
                auto diff = std::chrono::duration_cast<std::chrono::seconds>(
                    now - last_status_update).count();
                if (diff > 0) {
                    elapsed_time += diff;
                    if (elapsed_time > total_time) elapsed_time = total_time;
                    last_status_update = now;
                }
            }
        }

        struct TrackInfo {
            std::string title, artist, file, track_num;
        };

        std::vector<std::string> getAlbums() {
            std::vector<std::string> albums;
            struct mpd_connection* ctrl = mpd_connection_new(host.c_str(), port, 2000);
            if (!ctrl || mpd_connection_get_error(ctrl) != MPD_ERROR_SUCCESS) {
                if (ctrl) mpd_connection_free(ctrl);
                return albums;
            }
            if (mpd_search_db_tags(ctrl, MPD_TAG_ALBUM) && mpd_search_commit(ctrl)) {
                struct mpd_pair* pair;
                while ((pair = mpd_recv_pair_tag(ctrl, MPD_TAG_ALBUM))) {
                    if (pair->value && strlen(pair->value) > 0)
                        albums.push_back(pair->value);
                    mpd_return_pair(ctrl, pair);
                }
            }
            mpd_response_finish(ctrl);
            mpd_connection_free(ctrl);
            std::sort(albums.begin(), albums.end());
            return albums;
        }

        std::vector<TrackInfo> getAlbumTracks(const std::string& album) {
            std::vector<TrackInfo> tracks;
            struct mpd_connection* ctrl = mpd_connection_new(host.c_str(), port, 2000);
            if (!ctrl || mpd_connection_get_error(ctrl) != MPD_ERROR_SUCCESS) {
                if (ctrl) mpd_connection_free(ctrl);
                return tracks;
            }
            mpd_search_db_songs(ctrl, true);
            mpd_search_add_tag_constraint(ctrl, MPD_OPERATOR_DEFAULT, MPD_TAG_ALBUM, album.c_str());
            if (mpd_search_commit(ctrl)) {
                struct mpd_song* song;
                while ((song = mpd_recv_song(ctrl))) {
                    TrackInfo ti;
                    const char* t = mpd_song_get_tag(song, MPD_TAG_TITLE, 0);
                    const char* a = mpd_song_get_tag(song, MPD_TAG_ARTIST, 0);
                    const char* tr = mpd_song_get_tag(song, MPD_TAG_TRACK, 0);
                    ti.title = t ? t : mpd_song_get_uri(song);
                    ti.artist = a ? a : "";
                    ti.file = mpd_song_get_uri(song);
                    ti.track_num = tr ? tr : "";
                    size_t slash = ti.track_num.find('/');
                    if (slash != std::string::npos)
                        ti.track_num = ti.track_num.substr(0, slash);
                    tracks.push_back(ti);
                    mpd_song_free(song);
                }
            }
            mpd_response_finish(ctrl);
            mpd_connection_free(ctrl);
            return tracks;
        }

        void addAlbumToQueue(const std::string& album) {
            withMPD([&](auto* c) {
                mpd_search_add_db_songs(c, true);
                mpd_search_add_tag_constraint(c, MPD_OPERATOR_DEFAULT, MPD_TAG_ALBUM, album.c_str());
                mpd_search_commit(c); mpd_response_finish(c);
            });
        }

        void clearAndPlayAlbum(const std::string& album) {
            withMPD([&](auto* c) {
                mpd_run_clear(c);
                mpd_search_add_db_songs(c, true);
                mpd_search_add_tag_constraint(c, MPD_OPERATOR_DEFAULT, MPD_TAG_ALBUM, album.c_str());
                mpd_search_commit(c); mpd_response_finish(c);
                mpd_run_play(c);
            });
        }

        void addTrackToQueue(const std::string& file) {
            withMPD([&](auto* c) { mpd_run_add(c, file.c_str()); });
        }

        void setVolume(int delta) {
            withMPD([&](auto* c) {
                struct mpd_status* status = mpd_run_status(c);
                if (status) {
                    int vol = std::clamp((int)mpd_status_get_volume(status) + delta, 0, 100);
                    mpd_status_free(status);
                    mpd_run_set_volume(c, vol);
                }
            });
        }

        int getVolume() {
            int vol = -1;
            withMPD([&](auto* c) {
                struct mpd_status* status = mpd_run_status(c);
                if (status) { vol = (int)mpd_status_get_volume(status); mpd_status_free(status); }
            });
            return vol;
        }
};

// --- Bluetooth Monitor ---
class BluetoothMonitor {
    private:
        DBusConnection* conn;
        std::thread monitor_thread;
        std::atomic<bool> running{false};
        std::mutex data_mutex;

        std::atomic<bool> bt_connected{false};
        std::atomic<bool> bt_playing{false};

        std::string bt_title, bt_artist, bt_album;
        uint32_t bt_duration{0}, bt_position{0};

        std::function<void(bool)> on_connection_change;
        std::chrono::steady_clock::time_point last_position_update;

        void monitorLoop() {
            while (running) {
                dbus_connection_read_write(conn, 100);
                DBusMessage* msg = dbus_connection_pop_message(conn);
                if (!msg) continue;

                if (dbus_message_is_signal(msg, "org.freedesktop.DBus.Properties", "PropertiesChanged")) {
                    const char* interface = nullptr;
                    DBusMessageIter args;
                    if (dbus_message_iter_init(msg, &args) &&
                        dbus_message_iter_get_arg_type(&args) == DBUS_TYPE_STRING) {
                        dbus_message_iter_get_basic(&args, &interface);
                        if (interface) {
                            if (strcmp(interface, "org.bluez.MediaPlayer1") == 0)
                                parseMediaPlayer(msg);
                            else if (strcmp(interface, "org.bluez.MediaTransport1") == 0)
                                parseTransport(msg);
                        }
                    }
                }
                dbus_message_unref(msg);
            }
        }

        void parseMediaPlayer(DBusMessage* msg) {
            DBusMessageIter args, dict, entry, variant;
            if (!dbus_message_iter_init(msg, &args)) return;
            dbus_message_iter_next(&args);
            if (dbus_message_iter_get_arg_type(&args) != DBUS_TYPE_ARRAY) return;
            dbus_message_iter_recurse(&args, &dict);

            std::lock_guard<std::mutex> lock(data_mutex);
            while (dbus_message_iter_get_arg_type(&dict) == DBUS_TYPE_DICT_ENTRY) {
                dbus_message_iter_recurse(&dict, &entry);
                const char* key;
                dbus_message_iter_get_basic(&entry, &key);
                dbus_message_iter_next(&entry);
                dbus_message_iter_recurse(&entry, &variant);

                if (strcmp(key, "Status") == 0) {
                    const char* status;
                    dbus_message_iter_get_basic(&variant, &status);
                    bt_playing = (strcmp(status, "playing") == 0);
                } else if (strcmp(key, "Track") == 0) {
                    parseTrack(&variant);
                } else if (strcmp(key, "Position") == 0) {
                    dbus_message_iter_get_basic(&variant, &bt_position);
                    last_position_update = std::chrono::steady_clock::now();
                }
                dbus_message_iter_next(&dict);
            }
        }

        void parseTrack(DBusMessageIter* variant) {
            DBusMessageIter dict, entry, value;
            if (dbus_message_iter_get_arg_type(variant) != DBUS_TYPE_ARRAY) return;
            dbus_message_iter_recurse(variant, &dict);

            while (dbus_message_iter_get_arg_type(&dict) == DBUS_TYPE_DICT_ENTRY) {
                dbus_message_iter_recurse(&dict, &entry);
                const char* key;
                dbus_message_iter_get_basic(&entry, &key);
                dbus_message_iter_next(&entry);
                dbus_message_iter_recurse(&entry, &value);

                const char* str_val;
                if (strcmp(key, "Title") == 0) {
                    dbus_message_iter_get_basic(&value, &str_val);
                    bt_title = str_val ? str_val : "";
                } else if (strcmp(key, "Artist") == 0) {
                    dbus_message_iter_get_basic(&value, &str_val);
                    bt_artist = str_val ? str_val : "";
                } else if (strcmp(key, "Album") == 0) {
                    dbus_message_iter_get_basic(&value, &str_val);
                    bt_album = str_val ? str_val : "";
                } else if (strcmp(key, "Duration") == 0) {
                    dbus_message_iter_get_basic(&value, &bt_duration);
                }
                dbus_message_iter_next(&dict);
            }
        }

        void parseTransport(DBusMessage* msg) {
            DBusMessageIter args, dict, entry, variant;
            if (!dbus_message_iter_init(msg, &args)) return;
            dbus_message_iter_next(&args);
            if (dbus_message_iter_get_arg_type(&args) != DBUS_TYPE_ARRAY) return;
            dbus_message_iter_recurse(&args, &dict);

            while (dbus_message_iter_get_arg_type(&dict) == DBUS_TYPE_DICT_ENTRY) {
                dbus_message_iter_recurse(&dict, &entry);
                const char* key;
                dbus_message_iter_get_basic(&entry, &key);
                dbus_message_iter_next(&entry);
                dbus_message_iter_recurse(&entry, &variant);

                if (strcmp(key, "State") == 0) {
                    const char* state;
                    dbus_message_iter_get_basic(&variant, &state);
                    bool was_connected = bt_connected;
                    bt_connected = (strcmp(state, "active") == 0);
                    if (bt_connected != was_connected && on_connection_change)
                        on_connection_change(bt_connected);
                }
                dbus_message_iter_next(&dict);
            }
        }

    public:
        BluetoothMonitor() : conn(nullptr) {
            last_position_update = std::chrono::steady_clock::now();
        }
        ~BluetoothMonitor() { stop(); }

        bool start(std::function<void(bool)> connection_callback = nullptr) {
            on_connection_change = connection_callback;
            DBusError err;
            dbus_error_init(&err);
            conn = dbus_bus_get(DBUS_BUS_SYSTEM, &err);
            if (dbus_error_is_set(&err)) {
                printf("D-Bus connection error: %s\n", err.message);
                dbus_error_free(&err); return false;
            }
            dbus_bus_add_match(conn,
                "type='signal',interface='org.freedesktop.DBus.Properties',member='PropertiesChanged'", &err);
            if (dbus_error_is_set(&err)) {
                printf("D-Bus match error: %s\n", err.message);
                dbus_error_free(&err); return false;
            }
            running = true;
            monitor_thread = std::thread(&BluetoothMonitor::monitorLoop, this);
            printf("Bluetooth monitor started\n");
            return true;
        }

        void stop() {
            running = false;
            if (monitor_thread.joinable()) monitor_thread.join();
            if (conn) { dbus_connection_unref(conn); conn = nullptr; }
        }

        bool isConnected() const { return bt_connected; }
        bool isPlaying() const { return bt_playing; }
        std::string getTitle()  { std::lock_guard<std::mutex> lock(data_mutex); return bt_title; }
        std::string getArtist() { std::lock_guard<std::mutex> lock(data_mutex); return bt_artist; }
        std::string getAlbum()  { std::lock_guard<std::mutex> lock(data_mutex); return bt_album; }

        std::string getFormattedText() {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (!bt_connected) return "";
            if (bt_title.empty()) return "Bluetooth: Connected";
            std::string text = bt_title;
            if (!bt_artist.empty()) text += " - " + bt_artist;
            return text;
        }

        void updateElapsedTime() {
            std::lock_guard<std::mutex> lock(data_mutex);
            if (bt_playing && bt_duration > 0) {
                auto now = std::chrono::steady_clock::now();
                auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_position_update).count();
                if (diff > 0) {
                    bt_position += diff;
                    if (bt_position > bt_duration) bt_position = bt_duration;
                    last_position_update = now;
                }
            }
        }

        uint32_t getDuration() const { return bt_duration; }
        uint32_t getPosition() const { return bt_position; }
};

// --- Font Manager ---
class FontManager {
    private:
        FT_Library library;
        FT_Face face_regular, face_small, face_large;
        bool initialized;

        // Shared UTF-8 decoder
        static int decodeUTF8(const unsigned char*& utext, unsigned int& codepoint) {
            if ((*utext & 0x80) == 0x00) { codepoint = *utext; return 1; }
            if ((*utext & 0xE0) == 0xC0) {
                codepoint = (*utext & 0x1F) << 6 | (*(utext+1) & 0x3F); return 2;
            }
            if ((*utext & 0xF0) == 0xE0) {
                codepoint = (*utext & 0x0F) << 12 | (*(utext+1) & 0x3F) << 6 | (*(utext+2) & 0x3F); return 3;
            }
            if ((*utext & 0xF8) == 0xF0) {
                codepoint = (*utext & 0x07) << 18 | (*(utext+1) & 0x3F) << 12
                          | (*(utext+2) & 0x3F) << 6 | (*(utext+3) & 0x3F); return 4;
            }
            return 0; // invalid
        }

    public:
        enum FontSize { SMALL, REGULAR, LARGE };

        FontManager() : library(nullptr), face_regular(nullptr), face_small(nullptr),
                    face_large(nullptr), initialized(false) {}

        ~FontManager() {
            if (initialized) {
                if (face_large) FT_Done_Face(face_large);
                if (face_small) FT_Done_Face(face_small);
                if (face_regular) FT_Done_Face(face_regular);
                if (library) FT_Done_FreeType(library);
            }
        }

        bool init(const char* font_path) {
            if (FT_Init_FreeType(&library)) return false;
            if (FT_New_Face(library, font_path, 0, &face_regular)) {
                FT_Done_FreeType(library); return false;
            }
            FT_Set_Pixel_Sizes(face_regular, 0, 10);
            if (!FT_New_Face(library, font_path, 0, &face_small)) FT_Set_Pixel_Sizes(face_small, 0, 8);
            if (!FT_New_Face(library, font_path, 0, &face_large)) FT_Set_Pixel_Sizes(face_large, 0, 14);
            initialized = true;
            return true;
        }

        bool renderText(const char* text, uint8_t* buffer, int buf_width, int buf_height,
                    int x, int y, FontSize size = REGULAR, bool invert = false) {
            if (!initialized || !text) return false;

            FT_Face face = face_regular;
            if (size == SMALL && face_small) face = face_small;
            if (size == LARGE && face_large) face = face_large;

            int cursor_x = x;
            const unsigned char* utext = (const unsigned char*)text;

            while (*utext) {
                unsigned int codepoint;
                int bytes = decodeUTF8(utext, codepoint);
                if (bytes == 0) { utext++; continue; }

                if (FT_Load_Char(face, codepoint, FT_LOAD_RENDER)) { utext += bytes; continue; }

                FT_GlyphSlot slot = face->glyph;
                FT_Bitmap* bitmap = &slot->bitmap;
                int start_x = cursor_x + slot->bitmap_left;
                int start_y = y - slot->bitmap_top;

                for (unsigned int row = 0; row < bitmap->rows; row++) {
                    for (unsigned int col = 0; col < bitmap->width; col++) {
                        int px = start_x + col;
                        int py = start_y + row;
                        if (px >= 0 && px < buf_width && py >= 0 && py < buf_height) {
                            uint8_t gray = bitmap->buffer[row * bitmap->pitch + col];
                            if (gray > Config::SCREEN_WIDTH) {
                                int page = py / 8;
                                int bit = py % 8;
                                if (invert) buffer[page * buf_width + px] &= ~(1 << bit);
                                else        buffer[page * buf_width + px] |= (1 << bit);
                            }
                        }
                    }
                }
                cursor_x += slot->advance.x >> 6;
                utext += bytes;
            }
            return true;
        }

        int getTextWidth(const char* text, FontSize size = REGULAR) {
            if (!initialized || !text) return 0;
            FT_Face face = face_regular;
            if (size == SMALL && face_small) face = face_small;

            int width = 0;
            const unsigned char* utext = (const unsigned char*)text;
            while (*utext) {
                unsigned int codepoint;
                int bytes = decodeUTF8(utext, codepoint);
                if (bytes == 0) { utext++; continue; }
                if (!FT_Load_Char(face, codepoint, FT_LOAD_DEFAULT))
                    width += face->glyph->advance.x >> 6;
                utext += bytes;
            }
            return width;
        }

        bool isInitialized() const { return initialized; }
};

// --- Display Class ---
class Display {
    private:
        uint8_t _cs, _dc, _rst;
        FontManager* font_manager;
        bool is_sleeping = false;

    public:
    uint8_t buffer[Config::SPI_CHUNK_SIZE];

    Display(uint8_t cs, uint8_t dc, uint8_t rst) : _cs(cs), _dc(dc), _rst(rst), font_manager(nullptr) {
        memset(buffer, 0x00, sizeof(buffer));
    }

    bool begin() {
        // On bcm2835, CS is manual GPIO — must claim as output
        // On lgpio, CS is managed by SPI hardware — do NOT claim
        if (!hw->supportsAlerts()) {
            hw->claimOutput(_cs, 1);
        }
        hw->claimOutput(_dc, 0);
        hw->claimOutput(_rst, 1);

        hw->digitalWrite(_rst, 0);
        hw->delay(10);
        hw->digitalWrite(_rst, 1);
        hw->delay(10);

        const uint8_t init_cmds[] = {
            0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
            0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4,
            0xD3, 0x00, 0xD5, 0x70, 0xD9, 0x22, 0xDA, 0x12,
            0xDB, 0x34, 0x8D, 0x14, 0xAF
        };
        for (uint8_t cmd : init_cmds) sendCommand(cmd);
        return true;
    }

    void sendCommand(uint8_t cmd) {
        hw->digitalWrite(_dc, 0);
        hw->spiTransfer(_cs, cmd);
    }

    void display() {
        sendCommand(0xB0); sendCommand(0x00); sendCommand(0x10);
        hw->digitalWrite(_dc, 1);
        hw->spiWrite(_cs, (char*)buffer, sizeof(buffer));
    }

    void clear() { memset(buffer, 0x00, sizeof(buffer)); }
    void sleep() { sendCommand(0xAE); is_sleeping = true; }
    void shutdown() { sendCommand(0xAE); clear(); display(); }
    void wake() { sendCommand(0xAF); is_sleeping = false; }

    void drawPixel(int x, int y) {
        if (x < 0 || x >= Config::SCREEN_WIDTH || y < 0 || y >= 64) return;
        buffer[(y / 8) * Config::SCREEN_WIDTH + x] |= (1 << (y % 8));
    }

    void drawLine(int x0, int y0, int x1, int y1) {
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while (true) {
            drawPixel(x0, y0);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }

    void drawRect(int x, int y, int w, int h, bool filled = false) {
        if (filled) {
            int x_start = std::max(0, x), y_start = std::max(0, y);
            int x_end = std::min(Config::SCREEN_WIDTH, x + w);
            int y_end = std::min(Config::SCREEN_HEIGHT, y + h);
            for (int py = y_start; py < y_end; py++) {
                int row_offset = (py / 8) * Config::SCREEN_WIDTH;
                uint8_t bit_mask = (1 << (py % 8));
                for (int px = x_start; px < x_end; px++)
                    buffer[row_offset + px] |= bit_mask;
            }
        } else {
            drawLine(x, y, x+w-1, y); drawLine(x+w-1, y, x+w-1, y+h-1);
            drawLine(x+w-1, y+h-1, x, y+h-1); drawLine(x, y+h-1, x, y);
        }
    }

    void drawCircle(int x0, int y0, int radius, bool filled = false) {
        int x = radius, y = 0, err = 0;
        while (x >= y) {
            if (filled) {
                drawLine(x0-x, y0+y, x0+x, y0+y); drawLine(x0-x, y0-y, x0+x, y0-y);
                drawLine(x0-y, y0+x, x0+y, y0+x); drawLine(x0-y, y0-x, x0+y, y0-x);
            } else {
                drawPixel(x0+x, y0+y); drawPixel(x0+y, y0+x);
                drawPixel(x0-y, y0+x); drawPixel(x0-x, y0+y);
                drawPixel(x0-x, y0-y); drawPixel(x0-y, y0-x);
                drawPixel(x0+y, y0-x); drawPixel(x0+x, y0-y);
            }
            if (err <= 0) { y += 1; err += 2*y + 1; }
            if (err > 0) { x -= 1; err -= 2*x + 1; }
        }
    }

    void setFont(FontManager* fm) { font_manager = fm; }

    void drawText(uint8_t x, uint8_t y, const char* text, FontManager::FontSize size = FontManager::REGULAR) {
        if (font_manager) font_manager->renderText(text, buffer, Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, x, y, size);
    }
};

// Control State
struct ControlState {
    std::atomic<bool> running{true};
    std::atomic<int> current_viz{0};
    std::atomic<bool> is_sleeping{false};
};

// Audio Processor with sleep detection
class AudioProcessor {
    private:
        static constexpr int SAMPLE_RATE = 44100;
        static constexpr int CHANNELS = 2;
        static constexpr int FRAMES_PER_BUFFER = 2048;
        static constexpr int FFT_SIZE_BASS = 8192;
        static constexpr int FFT_SIZE_MID = 2048;
        static constexpr int FFT_SIZE_TREBLE = 512;
        static constexpr float SILENCE_THRESHOLD = 0.001f;
        static constexpr int SLEEP_TIMEOUT_SEC = 10;

        float* temp_left;
        float* temp_right;

        struct FreqBand { int low, high; float correction; };

        static constexpr FreqBand FREQ_BANDS[7] = {
            {63, 120, 0.7f}, {120, 350, 1.2f}, {350, 900, 2.0f},
            {900, 2000, 6.0f}, {2000, 5000, 11.0f},
            {5000, 10000, 30.0f}, {10000, 16000, 50.0f}
        };

        static constexpr FreqBand FREQ_BANDS_LARGE[64] = {
            {63,71,0.7f},{71,80,0.75f},{80,90,0.8f},{90,100,0.85f},
            {100,112,0.9f},{112,125,0.95f},{125,140,1.0f},{140,160,1.05f},
            {160,180,1.1f},{180,200,1.15f},{200,224,1.2f},{224,250,1.25f},
            {250,280,1.3f},{280,315,1.35f},{315,355,1.4f},{355,400,1.45f},
            {400,450,1.5f},{450,500,1.6f},{500,560,1.7f},{560,630,1.8f},
            {630,710,1.9f},{710,800,2.0f},{800,900,2.2f},{900,1000,2.4f},
            {1000,1120,2.6f},{1120,1250,2.8f},{1250,1400,3.0f},{1400,1600,3.3f},
            {1600,1800,3.6f},{1800,2000,4.0f},{2000,2240,4.4f},{2240,2500,4.8f},
            {2500,2800,5.3f},{2800,3150,5.8f},{3150,3550,6.4f},{3550,4000,7.0f},
            {4000,4500,7.7f},{4500,5000,8.5f},{5000,5600,9.3f},{5600,6300,10.2f},
            {6300,7100,11.2f},{7100,8000,12.3f},{8000,9000,13.5f},{9000,10000,14.8f},
            {10000,10600,16.2f},{10600,11200,17.7f},{11200,11800,19.4f},{11800,12500,21.2f},
            {12500,13200,23.2f},{13200,13900,25.4f},{13900,14600,27.8f},{14600,15000,30.4f},
            {15000,15200,33.3f},{15200,15400,36.4f},{15400,15550,39.8f},{15550,15700,41.5f},
            {15700,15800,43.3f},{15800,15875,45.1f},{15875,15925,47.0f},{15925,15962,48.5f},
            {15962,15981,49.3f},{15981,15990,49.6f},{15990,15995,49.8f},{15995,16000,50.0f}
        };

        snd_pcm_t* pcm_handle;
        std::thread audio_thread;
        std::atomic<bool> thread_running;
        std::atomic<bool> is_sleeping{false};

        float* circular_buffer_left;
        float* circular_buffer_right;
        std::atomic<size_t> write_pos{0};
        std::mutex buffer_mutex;

        fftwf_plan plan_bass, plan_mid, plan_treble;
        float *fft_in_bass, *fft_in_mid, *fft_in_treble;
        fftwf_complex *fft_out_bass, *fft_out_mid, *fft_out_treble;
        float *window_bass, *window_mid, *window_treble;

        std::array<float, 7> prev_left_spectrum{}, prev_right_spectrum{};
        std::array<float, 64> prev_left_spectrum_large{}, prev_right_spectrum_large{};

        float noise_reduction = 77.0f, sensitivity = 100.0f;
        float integral_factor, gravity_factor, scale_factor;

        std::chrono::steady_clock::time_point last_audio_time;
        std::atomic<float> max_amplitude{0.0f};

        void createHannWindow(float* window, int size) {
            for (int i = 0; i < size; i++)
                window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (size - 1)));
        }

        void audioThreadFunc() {
            int16_t* audio_buffer = new int16_t[FRAMES_PER_BUFFER * CHANNELS];

            while (thread_running) {
                int frames_to_read = is_sleeping ? 256 : FRAMES_PER_BUFFER;
                int frames = snd_pcm_readi(pcm_handle, audio_buffer, frames_to_read);
                if (frames < 0) {
                    frames = snd_pcm_recover(pcm_handle, frames, 0);
                    if (frames < 0) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
                }

                float frame_max = 0.0f;

                if (is_sleeping) {
                    for (int i = 0; i < frames * CHANNELS; i++)
                        frame_max = std::max(frame_max, std::abs(audio_buffer[i] / 32768.0f));
                    max_amplitude = frame_max;
                    if (frame_max > SILENCE_THRESHOLD)
                        last_audio_time = std::chrono::steady_clock::now();
                    continue;
                }

                std::lock_guard<std::mutex> lock(buffer_mutex);
                size_t pos = write_pos;
                for (int i = 0; i < frames; i++) {
                    circular_buffer_left[pos] = audio_buffer[i * CHANNELS] / 32768.0f;
                    circular_buffer_right[pos] = (CHANNELS > 1) ?
                        audio_buffer[i * CHANNELS + 1] / 32768.0f : circular_buffer_left[pos];
                    frame_max = std::max(frame_max, std::max(
                        std::abs(circular_buffer_left[pos]), std::abs(circular_buffer_right[pos])));
                    pos = (pos + 1) % (FFT_SIZE_BASS * 2);
                }
                write_pos = pos;
                max_amplitude = frame_max;
                if (frame_max > SILENCE_THRESHOLD)
                    last_audio_time = std::chrono::steady_clock::now();
            }
            delete[] audio_buffer;
        }

        void updateParameters() {
            float nr = noise_reduction / 100.0f;
            integral_factor = nr * 0.95f;
            gravity_factor = std::max(0.2f, 1.0f - nr * 0.8f);
            scale_factor = (sensitivity / 100.0f) * 1.8f;
        }

        void fillTempBuffers() {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            size_t read_pos = (write_pos + FFT_SIZE_BASS * 2 - FFT_SIZE_BASS) % (FFT_SIZE_BASS * 2);
            for (int i = 0; i < FFT_SIZE_BASS; i++) {
                temp_left[i] = circular_buffer_left[read_pos];
                temp_right[i] = circular_buffer_right[read_pos];
                read_pos = (read_pos + 1) % (FFT_SIZE_BASS * 2);
            }
        }

        void computeChannel(const float* samples, float* bands,
                            const FreqBand* freq_table, int num_bands,
                            int bass_end, int mid_end) {
            // BASS FFT
            for (int i = 0; i < FFT_SIZE_BASS; i++) fft_in_bass[i] = samples[i] * window_bass[i];
            fftwf_execute(plan_bass);
            for (int i = 0; i < bass_end; i++) {
                int lo = freq_table[i].low * FFT_SIZE_BASS / SAMPLE_RATE;
                int hi = freq_table[i].high * FFT_SIZE_BASS / SAMPLE_RATE;
                float sum = 0.0f;
                for (int j = lo; j < hi && j < FFT_SIZE_BASS/2; j++)
                    sum += fft_out_bass[j][0]*fft_out_bass[j][0] + fft_out_bass[j][1]*fft_out_bass[j][1];
                bands[i] = sqrtf(sum / (hi - lo)) * scale_factor * freq_table[i].correction;
            }
            // MID FFT
            for (int i = 0; i < FFT_SIZE_MID; i++) fft_in_mid[i] = samples[i] * window_mid[i];
            fftwf_execute(plan_mid);
            for (int i = bass_end; i < mid_end; i++) {
                int lo = freq_table[i].low * FFT_SIZE_MID / SAMPLE_RATE;
                int hi = freq_table[i].high * FFT_SIZE_MID / SAMPLE_RATE;
                float sum = 0.0f;
                for (int j = lo; j < hi && j < FFT_SIZE_MID/2; j++)
                    sum += fft_out_mid[j][0]*fft_out_mid[j][0] + fft_out_mid[j][1]*fft_out_mid[j][1];
                bands[i] = sqrtf(sum / (hi - lo)) * scale_factor * freq_table[i].correction;
            }
            // TREBLE FFT
            for (int i = 0; i < FFT_SIZE_TREBLE; i++) fft_in_treble[i] = samples[i] * window_treble[i];
            fftwf_execute(plan_treble);
            for (int i = mid_end; i < num_bands; i++) {
                int lo = freq_table[i].low * FFT_SIZE_TREBLE / SAMPLE_RATE;
                int hi = freq_table[i].high * FFT_SIZE_TREBLE / SAMPLE_RATE;
                if (hi <= lo) hi = lo + 1;
                float sum = 0.0f;
                for (int j = lo; j < hi && j < FFT_SIZE_TREBLE/2; j++)
                    sum += fft_out_treble[j][0]*fft_out_treble[j][0] + fft_out_treble[j][1]*fft_out_treble[j][1];
                bands[i] = sqrtf(sum / (hi - lo)) * scale_factor * freq_table[i].correction;
            }
        }

        template<int N>
        void applySmoothingAndOutput(float* prev, const float* bands, int* out) {
            for (int i = 0; i < N; i++) {
                float smoothed = integral_factor * prev[i] + (1.0f - integral_factor) * bands[i];
                if (smoothed < prev[i]) {
                    float fall = (prev[i] - smoothed) * gravity_factor;
                    prev[i] -= fall;
                    prev[i] = std::max(prev[i], smoothed);
                } else {
                    prev[i] = smoothed;
                }
                out[i] = std::clamp((int)prev[i], 0, 255);
            }
        }

    public:
        AudioProcessor() : pcm_handle(nullptr), thread_running(false) {
            int buf_sz = FFT_SIZE_BASS * 2;
            circular_buffer_left = new float[buf_sz]();
            circular_buffer_right = new float[buf_sz]();
            temp_left = new float[FFT_SIZE_BASS];
            temp_right = new float[FFT_SIZE_BASS];

            fft_in_bass = (float*)fftwf_malloc(sizeof(float) * FFT_SIZE_BASS);
            fft_out_bass = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * (FFT_SIZE_BASS/2 + 1));
            plan_bass = fftwf_plan_dft_r2c_1d(FFT_SIZE_BASS, fft_in_bass, fft_out_bass, FFTW_ESTIMATE);

            fft_in_mid = (float*)fftwf_malloc(sizeof(float) * FFT_SIZE_MID);
            fft_out_mid = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * (FFT_SIZE_MID/2 + 1));
            plan_mid = fftwf_plan_dft_r2c_1d(FFT_SIZE_MID, fft_in_mid, fft_out_mid, FFTW_ESTIMATE);

            fft_in_treble = (float*)fftwf_malloc(sizeof(float) * FFT_SIZE_TREBLE);
            fft_out_treble = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * (FFT_SIZE_TREBLE/2 + 1));
            plan_treble = fftwf_plan_dft_r2c_1d(FFT_SIZE_TREBLE, fft_in_treble, fft_out_treble, FFTW_ESTIMATE);

            window_bass = new float[FFT_SIZE_BASS];
            window_mid = new float[FFT_SIZE_MID];
            window_treble = new float[FFT_SIZE_TREBLE];
            createHannWindow(window_bass, FFT_SIZE_BASS);
            createHannWindow(window_mid, FFT_SIZE_MID);
            createHannWindow(window_treble, FFT_SIZE_TREBLE);

            updateParameters();
            last_audio_time = std::chrono::steady_clock::now();
        }

        ~AudioProcessor() {
            stop();
            delete[] circular_buffer_left; delete[] circular_buffer_right;
            delete[] temp_left; delete[] temp_right;
            delete[] window_bass; delete[] window_mid; delete[] window_treble;
            fftwf_destroy_plan(plan_bass); fftwf_destroy_plan(plan_mid); fftwf_destroy_plan(plan_treble);
            fftwf_free(fft_in_bass); fftwf_free(fft_out_bass);
            fftwf_free(fft_in_mid); fftwf_free(fft_out_mid);
            fftwf_free(fft_in_treble); fftwf_free(fft_out_treble);
        }

        bool start() {
            int err = snd_pcm_open(&pcm_handle, "rav", SND_PCM_STREAM_CAPTURE, 0);
            if (err < 0) err = snd_pcm_open(&pcm_handle, "hw:Loopback,1", SND_PCM_STREAM_CAPTURE, 0);
            if (err < 0) return false;

            snd_pcm_hw_params_t* hw_params;
            snd_pcm_hw_params_alloca(&hw_params);
            snd_pcm_hw_params_any(pcm_handle, hw_params);
            snd_pcm_hw_params_set_access(pcm_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
            snd_pcm_hw_params_set_format(pcm_handle, hw_params, SND_PCM_FORMAT_S16_LE);
            snd_pcm_hw_params_set_channels(pcm_handle, hw_params, CHANNELS);
            snd_pcm_hw_params_set_buffer_size(pcm_handle, hw_params, 8192);
            snd_pcm_hw_params_set_period_size(pcm_handle, hw_params, 2048, 0);
            unsigned int rate = SAMPLE_RATE;
            snd_pcm_hw_params_set_rate_near(pcm_handle, hw_params, &rate, 0);

            if (snd_pcm_hw_params(pcm_handle, hw_params) < 0) {
                snd_pcm_close(pcm_handle); return false;
            }
            thread_running = true;
            audio_thread = std::thread(&AudioProcessor::audioThreadFunc, this);
            return true;
        }

        void setSleepState(bool sleeping) { is_sleeping = sleeping; }
        void resetSleepTimer() { last_audio_time = std::chrono::steady_clock::now(); }

        void stop() {
            if (thread_running) {
                thread_running = false;
                if (pcm_handle) snd_pcm_drop(pcm_handle);
                if (audio_thread.joinable()) audio_thread.join();
            }
            if (pcm_handle) { snd_pcm_close(pcm_handle); pcm_handle = nullptr; }
        }

        bool checkForAudio() {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - last_audio_time).count();
            return elapsed < SLEEP_TIMEOUT_SEC;
        }

        void getSpectrumData(std::array<int, 7>& left_out, std::array<int, 7>& right_out) {
            std::array<float, 7> lb{}, rb{};
            fillTempBuffers();
            computeChannel(temp_left, lb.data(), FREQ_BANDS, 7, 3, 5);
            computeChannel(temp_right, rb.data(), FREQ_BANDS, 7, 3, 5);
            applySmoothingAndOutput<7>(prev_left_spectrum.data(), lb.data(), left_out.data());
            applySmoothingAndOutput<7>(prev_right_spectrum.data(), rb.data(), right_out.data());
        }

        void getLargeSpectrumData(std::array<int, 64>& left_out, std::array<int, 64>& right_out) {
            std::array<float, 64> lb{}, rb{};
            fillTempBuffers();
            computeChannel(temp_left, lb.data(), FREQ_BANDS_LARGE, 64, 24, 44);
            computeChannel(temp_right, rb.data(), FREQ_BANDS_LARGE, 64, 24, 44);
            applySmoothingAndOutput<64>(prev_left_spectrum_large.data(), lb.data(), left_out.data());
            applySmoothingAndOutput<64>(prev_right_spectrum_large.data(), rb.data(), right_out.data());
        }

        void getVUMeterData(int& left_out, int& right_out) {
            std::array<int, 7> l, r;
            getSpectrumData(l, r);
            int ls = 0, rs = 0;
            for (int i = 0; i < 7; i++) { ls += l[i]; rs += r[i]; }
            left_out = ls / 7; right_out = rs / 7;
        }

        void getWaveformData(float* out, int samples, bool left_channel) {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            size_t read_pos = (write_pos + FFT_SIZE_BASS * 2 - samples) % (FFT_SIZE_BASS * 2);
            for (int i = 0; i < samples; i++) {
                out[i] = left_channel ? circular_buffer_left[read_pos] : circular_buffer_right[read_pos];
                read_pos = (read_pos + 1) % (FFT_SIZE_BASS * 2);
            }
        }

        void setSensitivity(int v) { sensitivity = std::clamp((float)v, 10.0f, 300.0f); updateParameters(); }
        void setNoiseReduction(int v) { noise_reduction = std::clamp((float)v, 0.0f, 100.0f); updateParameters(); }
        int getSensitivity() const { return (int)sensitivity; }
        int getNoiseReduction() const { return (int)noise_reduction; }
};

// --- TextScroller (Ping-Pong) ---
class TextScroller {
    private:
        std::string current_text;
        float scroll_position = 0;
        std::chrono::steady_clock::time_point last_update_time;
        float wait_timer = 0;
        int text_width_pixels = 0;
        static constexpr float SCROLL_SPEED = 25.0f;
        static constexpr float WAIT_TIME_SEC = 2.0f;
        enum State { WAIT_START, SCROLL_RIGHT, WAIT_END, SCROLL_LEFT } state = WAIT_START;

    public:
        TextScroller() { last_update_time = std::chrono::steady_clock::now(); }

        void setText(const std::string& text) {
            if (text != current_text) { current_text = text; reset(); }
        }

        void reset() {
            scroll_position = 0; wait_timer = 0; state = WAIT_START;
            text_width_pixels = 0;
            last_update_time = std::chrono::steady_clock::now();
        }

        void render(Display* display, int x_offset, int y_offset, int max_width, FontManager* fm) {
            if (current_text.empty() || !fm) return;
            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now - last_update_time).count();
            last_update_time = now;

            if (text_width_pixels == 0)
                text_width_pixels = fm->getTextWidth(current_text.c_str(), FontManager::SMALL);

            int render_x = x_offset;
            if (text_width_pixels <= max_width) {
                render_x = x_offset + (max_width - text_width_pixels) / 2;
                state = WAIT_START; scroll_position = 0;
            } else {
                float max_scroll = (float)(text_width_pixels - max_width);
                switch (state) {
                    case WAIT_START:
                        scroll_position = 0; wait_timer += dt;
                        if (wait_timer >= WAIT_TIME_SEC) { wait_timer = 0; state = SCROLL_RIGHT; }
                        break;
                    case SCROLL_RIGHT:
                        scroll_position += SCROLL_SPEED * dt;
                        if (scroll_position >= max_scroll) { scroll_position = max_scroll; state = WAIT_END; }
                        break;
                    case WAIT_END:
                        wait_timer += dt;
                        if (wait_timer >= WAIT_TIME_SEC) { wait_timer = 0; state = SCROLL_LEFT; }
                        break;
                    case SCROLL_LEFT:
                        scroll_position -= SCROLL_SPEED * dt;
                        if (scroll_position <= 0) { scroll_position = 0; state = WAIT_START; }
                        break;
                }
                render_x = x_offset - (int)scroll_position;
            }

            fm->renderText(current_text.c_str(), display->buffer, Config::SCREEN_WIDTH,
                        Config::SCREEN_HEIGHT, render_x, y_offset, FontManager::SMALL);
        }
};

// --- Visualization Base ---
class Visualization {
    protected:
        Display* left_display;
        Display* right_display;
        MPDClient* mpd_client;
        BluetoothMonitor* bluetooth_monitor;
        FontManager* font_manager;
        TextScroller title_scroller_left, title_scroller_right;

        void drawTitle(Display* display, int y, bool is_left) {
            if (!font_manager) return;
            std::string txt;
            if (bluetooth_monitor && bluetooth_monitor->isConnected())
                txt = bluetooth_monitor->getFormattedText();
            else if (mpd_client) txt = mpd_client->getFormattedText();
            if (txt.empty()) return;

            TextScroller& scroller = is_left ? title_scroller_left : title_scroller_right;
            scroller.setText(txt);
            scroller.render(display, 0, y, Config::SCREEN_WIDTH, font_manager);
        }

    public:
        Visualization(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f)
            : left_display(l), right_display(r), mpd_client(m), bluetooth_monitor(bt), font_manager(f) {}
        virtual ~Visualization() = default;
        virtual void render(ControlState& state, AudioProcessor& audio) = 0;
        virtual const char* getName() const = 0;
};

// --- Visualizations ---
// (VUMeterViz, SpectrumViz, EmptySpectrumViz, WaveformViz, TrackInfoViz, PlaylistEditorViz)
// These are identical to the RPi5 version — they only use Display/AudioProcessor/MPDClient APIs
// which are already hardware-abstracted. Including them verbatim for completeness.

class VUMeterViz : public Visualization {
    public:
        struct DBPosition { int x; const char* text; };
        std::array<DBPosition, 11> db_positions;
        static constexpr const char* POWER_SCALE[6] = {"0", "20", "40", "60", "80", "100"};

        void calculateDBPositions() {
            const float db_values[11] = {-20, -10, -7, -5, -3, -2, -1, 0, 1, 2, 3};
            for (int i = 0; i < 11; i++) {
                float value = powf(10.0f, db_values[i] / 20.0f);
                float log_pos = log10f(value);
                float min_log = log10f(powf(10.0f, -20.0f / 20.0f));
                float max_log = log10f(powf(10.0f, 3.0f / 20.0f));
                db_positions[i].x = (int)((log_pos - min_log) / (max_log - min_log) * 125.0f);
                static char texts[11][4];
                snprintf(texts[i], 4, "%d", abs((int)db_values[i]));
                db_positions[i].text = texts[i];
            }
        }

        VUMeterViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f)
            : Visualization(l, r, m, bt, f) { calculateDBPositions(); }
        const char* getName() const override { return "VU Meter"; }

        void render(ControlState&, AudioProcessor& audio) override {
            int l_vu, r_vu; audio.getVUMeterData(l_vu, r_vu);
            drawMeter(left_display, l_vu, "LEFT");
            drawMeter(right_display, r_vu, "RIGHT");
        }

        void drawMeter(Display* d, int val, const char* label) {
            d->clear();
            for (const auto& pos : db_positions) {
                d->drawText(pos.x, 5, pos.text, FontManager::SMALL);
                d->drawLine(pos.x, 7, pos.x, 9);
            }
            d->drawLine(108, 8, 127, 8); d->drawLine(0, 9, 127, 9); d->drawLine(0, 11, 110, 11);
            d->drawLine(0, 6, 0, 8); d->drawLine(0, 11, 0, 13); d->drawLine(127, 6, 127, 8);
            for (int i = 0; i < 6; i++) {
                d->drawText(i*22, 22, POWER_SCALE[i], FontManager::SMALL);
                d->drawLine(i*22, 11, i*22, 13);
            }
            d->drawText(0, 28, "-", FontManager::SMALL); d->drawText(124, 28, "+", FontManager::SMALL);
            d->drawText(0, 64, label, FontManager::SMALL); d->drawText(120, 64, "dB", FontManager::SMALL);

            int pos = (val * 127) / 255;
            int start_x = 71 - (127 - pos) / 8;
            int end_y = 20 - pos * (127 - pos) / 200;
            d->drawLine(start_x, 63, pos, end_y);
        }
};

class SpectrumViz : public Visualization {
    std::array<float, 7> peak_l{}, peak_r{};
    const char* LABELS[7] = {"63", "160", "400", "1K", "2.5K", "6.3K", "16K"};
    public:
        SpectrumViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l,r,m,bt,f) {}
        const char* getName() const override { return "Spectrum"; }
        void render(ControlState&, AudioProcessor& audio) override {
            std::array<int, 7> l, r; audio.getSpectrumData(l, r);
            drawSpec(left_display, l, peak_l, true); drawSpec(right_display, r, peak_r, false);
        }
        void drawSpec(Display* d, const std::array<int,7>& data, std::array<float,7>& peaks, bool is_l) {
            d->clear(); drawTitle(d, 5, is_l);
            for (int i = 0; i < 7; i++) {
                int x = 1+i*19, h = (data[i]*49)/255, y = 57-h;
                if (h > 0) d->drawRect(x, y, 12, h, true);
                if (y < peaks[i]) peaks[i] = y;
                else peaks[i] = std::min(56.0f, peaks[i] + 0.5f);
                d->drawLine(x, (int)peaks[i], x+11, (int)peaks[i]);
                d->drawText(x, 64, LABELS[i], FontManager::SMALL);
            }
        }
};

class EmptySpectrumViz : public Visualization {
    const char* LABELS[7] = {"63", "160", "400", "1K", "2.5K", "6.3K", "16K"};
    public:
        EmptySpectrumViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l,r,m,bt,f) {}
        const char* getName() const override { return "Empty Spectrum"; }
        void render(ControlState&, AudioProcessor& audio) override {
            std::array<int, 7> l, r; audio.getSpectrumData(l, r);
            drawSpec(left_display, l, true); drawSpec(right_display, r, false);
        }
        void drawSpec(Display* d, const std::array<int,7>& data, bool is_l) {
            d->clear(); drawTitle(d, 5, is_l);
            for (int i = 0; i < 7; i++) {
                int x = 1+i*19, h = (data[i]*49)/255, y = 57-h, w = 12;
                if (h > 0) { d->drawLine(x,57,x,y); d->drawLine(x+w,57,x+w,y); d->drawLine(x,y,x+w,y); }
                d->drawText(x, 64, LABELS[i], FontManager::SMALL);
            }
        }
};

class WaveformViz : public Visualization {
    float samples[128];
    public:
        WaveformViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l,r,m,bt,f) {}
        const char* getName() const override { return "Waveform"; }
        void render(ControlState&, AudioProcessor& audio) override {
            drawWave(left_display, audio, true); drawWave(right_display, audio, false);
        }
        void drawWave(Display* d, AudioProcessor& a, bool is_left) {
            d->clear(); drawTitle(d, 5, is_left);
            a.getWaveformData(samples, 128, is_left);
            int cy = 37; d->drawLine(0, cy, 127, cy);
            for (int i = 0; i < 127; i++) {
                int y1 = cy-(int)(samples[i]*25), y2 = cy-(int)(samples[i+1]*25);
                d->drawLine(i, std::clamp(y1,12,63), i+1, std::clamp(y2,12,63));
            }
        }
};

class TrackInfoViz : public Visualization {
    private:
        std::array<float, 7> peak_l{}, peak_r{};
        TextScroller info_scroller;
    public:
        TrackInfoViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l,r,m,bt,f) {}
        const char* getName() const override { return "Track Info"; }

        void render(ControlState& state, AudioProcessor& audio) override {
            left_display->clear();
            int y = 5;
            bool bt_active = bluetooth_monitor && bluetooth_monitor->isConnected();

            if (bt_active) {
                bluetooth_monitor->updateElapsedTime();
                info_scroller.setText(bluetooth_monitor->getFormattedText());
                info_scroller.render(left_display, 1, y, Config::SCREEN_WIDTH, font_manager); y += 10;
                left_display->drawText(1, y, "Source: Bluetooth", FontManager::SMALL); y += 10;
                left_display->drawText(1, y, "A2DP SBC", FontManager::SMALL); y += 20;
                uint32_t elapsed = bluetooth_monitor->getPosition()/1000;
                uint32_t total = bluetooth_monitor->getDuration()/1000;
                if (total > 0) {
                    char ts[32];
                    snprintf(ts, sizeof(ts), "%02u:%02u / %02u:%02u", elapsed/60, elapsed%60, total/60, total%60);
                    left_display->drawText(1, y, ts, FontManager::SMALL); y += 5;
                    int bw = 126, pw = (elapsed*bw)/total;
                    left_display->drawRect(1, y, bw, 5, false);
                    if (pw > 2) left_display->drawRect(2, y+1, pw-2, 3, true);
                }
            } else if (mpd_client) {
                mpd_client->updateElapsedTime();
                unsigned int elapsed = mpd_client->getElapsedTime(), total = mpd_client->getTotalTime();

                std::stringstream info_line;
                info_line << mpd_client->getTrackNumber() << ". " << mpd_client->getTitle();
                std::string art = mpd_client->getArtist();
                if (!art.empty() && art != "Unknown Artist") info_line << " - " << art;
                std::string alb = mpd_client->getAlbum();
                if (!alb.empty() && alb != "Unknown Album") info_line << " - " << alb;
                std::string dt = mpd_client->getDate();
                if (!dt.empty()) info_line << " (" << dt << ")";

                info_scroller.setText(info_line.str());
                info_scroller.render(left_display, 1, y, Config::SCREEN_WIDTH-2, font_manager); y += 10;

                left_display->drawText(1, y, ("Bitrate: "+mpd_client->getBitrate()).c_str(), FontManager::SMALL); y += 10;
                left_display->drawText(1, y, mpd_client->getFormat().c_str(), FontManager::SMALL); y += 20;
                left_display->drawText(1, y, (mpd_client->formatTime(elapsed)+" / "+mpd_client->formatTime(total)).c_str(), FontManager::SMALL); y += 5;

                if (total > 0) {
                    int bw = 126, pw = (elapsed*bw)/total;
                    left_display->drawRect(1, y, bw, 5, false);
                    if (pw > 2) left_display->drawRect(2, y+1, pw-2, 3, true);
                }
            }

            // Mini spectrum on left display
            std::array<int, 7> ls, rs;
            audio.getSpectrumData(ls, rs);
            for (int i = 0; i < 7; i++) {
                int xl = 80+i*3, hl = (ls[i]*32)/255;
                if (hl > 0) left_display->drawRect(xl, 43-hl, 2, hl, true);
                if (43-hl < peak_l[i]) peak_l[i] = 43-hl;
                else peak_l[i] = std::min(43.0f, peak_l[i]+0.5f);
                left_display->drawLine(xl, (int)peak_l[i], xl+1, (int)peak_l[i]);

                int xr = 103+i*3, hr = (rs[i]*32)/255;
                if (hr > 0) left_display->drawRect(xr, 43-hr, 2, hr, true);
                if (43-hr < peak_r[i]) peak_r[i] = 43-hr;
                else peak_r[i] = std::min(43.0f, peak_r[i]+0.5f);
                left_display->drawLine(xr, (int)peak_r[i], xr+1, (int)peak_r[i]);
            }
            left_display->drawLine(77, 13, 77, 45); left_display->drawLine(77, 45, 125, 45);

            // Right display: 64-band spectrum
            right_display->clear();
            std::array<int, 64> ls2, rs2;
            audio.getLargeSpectrumData(ls2, rs2);
            for (int i = 0; i < 64; i++) {
                int x = i*2;
                int hl = 1+(ls2[i]*31)/255;
                right_display->drawRect(x, 32-hl, 1, hl, true);
                int hr = 1+(rs2[i]*31)/255;
                right_display->drawRect(x, 32, 1, hr, true);
            }
        }
};

class PlaylistEditorViz : public Visualization {
    enum class EditorState { BROWSE_ALBUMS, BROWSE_TRACKS, FEEDBACK };
    EditorState editor_state = EditorState::BROWSE_ALBUMS;
    bool albums_loaded = false, loading = false;
    std::vector<std::string> albums;
    std::vector<MPDClient::TrackInfo> tracks;
    int album_cursor = 0, album_scroll = 0, track_cursor = 0, track_scroll = 0;
    std::string selected_album, feedback_msg;
    std::chrono::steady_clock::time_point feedback_ts;
    std::thread load_thread;
    std::mutex data_mtx;
    bool prev_up = false, prev_dn = false, prev_sel = false, prev_back = false;
    static constexpr int VISIBLE_LINES = 6, LINE_H = 9, LIST_Y0 = 18, HEADER_Y = 7;

    std::string truncatePx(const std::string& s, int max_px) {
        if (!font_manager) return s.substr(0, 20);
        if (font_manager->getTextWidth(s.c_str(), FontManager::SMALL) <= max_px) return s;
        std::string r = s;
        while (r.size() > 2 && font_manager->getTextWidth((r+"..").c_str(), FontManager::SMALL) > max_px) r.pop_back();
        return r + "..";
    }

    void drawSelectionRow(Display* d, int row, const std::string& label, bool sel) {
        int y = LIST_Y0 + row * LINE_H;
        if (sel) {
            d->drawRect(0, y-7, 126, 8, true);
            if (font_manager)
                font_manager->renderText(truncatePx(label,120).c_str(), d->buffer,
                    Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, 2, y, FontManager::SMALL, true);
        } else d->drawText(2, y, truncatePx(label,120).c_str(), FontManager::SMALL);
    }

    void drawScrollbar(Display* d, int total, int vis, int off) {
        if (total <= vis) return;
        int bt = LIST_Y0-7, bb = LIST_Y0+vis*LINE_H-1, bh = bb-bt;
        int th = std::max(4, bh*vis/total), ty = bt + bh*off/total;
        d->drawLine(127, bt, 127, bb); d->drawRect(126, ty, 2, th, true);
    }

    void setFeedback(const std::string& msg) {
        feedback_msg = msg; feedback_ts = std::chrono::steady_clock::now();
        editor_state = EditorState::FEEDBACK;
    }

    void startLoadAlbums() {
        if (loading) return;
        loading = true; albums_loaded = false;
        if (load_thread.joinable()) load_thread.join();
        load_thread = std::thread([this]() {
            auto res = mpd_client->getAlbums();
            { std::lock_guard<std::mutex> lk(data_mtx); albums = std::move(res); }
            albums_loaded = true; loading = false;
        });
    }

    void startLoadTracks(const std::string& alb) {
        if (loading) return;
        loading = true;
        if (load_thread.joinable()) load_thread.join();
        load_thread = std::thread([this, alb]() {
            auto res = mpd_client->getAlbumTracks(alb);
            { std::lock_guard<std::mutex> lk(data_mtx); tracks = std::move(res); }
            loading = false;
        });
    }

    void renderBrowser(Display* d) {
        d->clear();
        if (!albums_loaded && loading) { d->drawText(25, 35, "Chargement...", FontManager::SMALL); return; }
        if (editor_state == EditorState::FEEDBACK) {
            int ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - feedback_ts).count();
            if (ms > 1500) editor_state = EditorState::BROWSE_ALBUMS;
            d->drawRect(8,22,112,20,false); d->drawRect(9,23,110,18,false);
            int tw = font_manager ? font_manager->getTextWidth(feedback_msg.c_str(), FontManager::SMALL) : 0;
            d->drawText((128-tw)/2, 35, feedback_msg.c_str(), FontManager::SMALL);
            return;
        }
        if (editor_state == EditorState::BROWSE_ALBUMS) {
            d->drawText(0, HEADER_Y, "BIBLIOTHEQUE", FontManager::SMALL);
            d->drawLine(0, 9, 127, 9);
            std::lock_guard<std::mutex> lk(data_mtx);
            if (albums.empty()) { d->drawText(10,35,"Aucun album",FontManager::SMALL); return; }
            for (int i = 0; i < VISIBLE_LINES && album_scroll+i < (int)albums.size(); i++)
                drawSelectionRow(d, i, albums[album_scroll+i], album_scroll+i == album_cursor);
            drawScrollbar(d, (int)albums.size(), VISIBLE_LINES, album_scroll);
        } else if (editor_state == EditorState::BROWSE_TRACKS) {
            d->drawText(0, HEADER_Y, truncatePx(selected_album,118).c_str(), FontManager::SMALL);
            d->drawLine(0, 9, 127, 9);
            if (loading) { d->drawText(25,35,"Chargement...",FontManager::SMALL); return; }
            std::lock_guard<std::mutex> lk(data_mtx);
            int total = 2+(int)tracks.size();
            for (int i = 0; i < VISIBLE_LINES && track_scroll+i < total; i++) {
                int idx = track_scroll+i;
                std::string lbl;
                if (idx == 0) lbl = "> Jouer album";
                else if (idx == 1) lbl = "+ Ajouter album";
                else { auto& t = tracks[idx-2]; lbl = t.track_num.empty() ? t.title : t.track_num+". "+t.title; }
                drawSelectionRow(d, i, lbl, idx == track_cursor);
            }
            drawScrollbar(d, total, VISIBLE_LINES, track_scroll);
        }
    }

    void renderInfo(Display* d, AudioProcessor& audio) {
        d->clear();
        std::string t = mpd_client ? mpd_client->getTitle() : "";
        std::string a = mpd_client ? mpd_client->getArtist() : "";
        d->drawText(1, 7, truncatePx(t,126).c_str(), FontManager::SMALL);
        d->drawText(1, 16, truncatePx(a,126).c_str(), FontManager::SMALL);
        d->drawLine(0, 18, 127, 18);

        std::array<int, 64> lsp, rsp;
        audio.getLargeSpectrumData(lsp, rsp);
        for (int i = 0; i < 64; i++) {
            int x = i*2;
            d->drawRect(x, 40-1-(lsp[i]*20)/255, 1, 1+(lsp[i]*20)/255, true);
            d->drawRect(x, 41, 1, 1+(rsp[i]*20)/255, true);
        }
        d->drawLine(0, 40, 127, 40);

        if (editor_state == EditorState::BROWSE_ALBUMS)
            d->drawText(0, 64, "SEL:entrer BCK:quitter", FontManager::SMALL);
        else d->drawText(0, 64, "SEL:ajouter BCK:retour", FontManager::SMALL);
    }

    public:
    PlaylistEditorViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l,r,m,bt,f) {}
    ~PlaylistEditorViz() { if (load_thread.joinable()) load_thread.join(); }
    const char* getName() const override { return "Playlist Editor"; }

    void onActivate() {
        if (!albums_loaded && !loading) startLoadAlbums();
        prev_sel = false; prev_back = false;
    }

    bool handleInput(bool btn_up, bool btn_dn, bool btn_sel, bool btn_back) {
        bool want_exit = false;
        if (editor_state == EditorState::FEEDBACK) {
            if (btn_sel || btn_back) editor_state = EditorState::BROWSE_ALBUMS;
            return false;
        }
        if (editor_state == EditorState::BROWSE_ALBUMS) {
            std::lock_guard<std::mutex> lk(data_mtx);
            int n = (int)albums.size();
            if (btn_back) want_exit = true;
            if (!loading && n > 0) {
                if (btn_dn) { album_cursor = std::min(album_cursor+1, n-1);
                    if (album_cursor >= album_scroll+VISIBLE_LINES) album_scroll = album_cursor-VISIBLE_LINES+1; }
                if (btn_up) { album_cursor = std::max(album_cursor-1, 0);
                    if (album_cursor < album_scroll) album_scroll = album_cursor; }
                if (btn_sel && album_cursor < n) {
                    selected_album = albums[album_cursor];
                    track_cursor = 0; track_scroll = 0;
                    editor_state = EditorState::BROWSE_TRACKS;
                    startLoadTracks(selected_album);
                }
            }
        } else if (editor_state == EditorState::BROWSE_TRACKS) {
            if (btn_back) { editor_state = EditorState::BROWSE_ALBUMS; return false; }
            if (loading) return false;
            std::lock_guard<std::mutex> lk(data_mtx);
            int total = 2+(int)tracks.size();
            if (btn_dn) { track_cursor = std::min(track_cursor+1, total-1);
                if (track_cursor >= track_scroll+VISIBLE_LINES) track_scroll = track_cursor-VISIBLE_LINES+1; }
            if (btn_up) { track_cursor = std::max(track_cursor-1, 0);
                if (track_cursor < track_scroll) track_scroll = track_cursor; }
            if (btn_sel) {
                if (track_cursor == 0) { mpd_client->clearAndPlayAlbum(selected_album); setFeedback("  Lecture !"); }
                else if (track_cursor == 1) { mpd_client->addAlbumToQueue(selected_album); setFeedback("  Album ajoute !"); }
                else { int ti = track_cursor-2;
                    if (ti < (int)tracks.size()) { mpd_client->addTrackToQueue(tracks[ti].file); setFeedback("  Piste ajoutee !"); } }
            }
        }
        return want_exit;
    }

    void render(ControlState&, AudioProcessor& audio) override {
        renderBrowser(left_display);
        renderInfo(right_display, audio);
    }
};

// ============================================================
// Rotary Encoder — uses HAL alerts on lgpio, polling on bcm2835
// ============================================================

class RotaryEncoder {
    int clk_pin_, dt_pin_, sw_pin_;
    uint8_t state_ = 0;
    int accumulator_ = 0;
    int detent_threshold_ = 4;

    std::atomic<int> rotation_{0};
    std::atomic<bool> button_pressed_{false};

    uint64_t btn_last_ts_ = 0;
    static constexpr uint64_t BTN_DEBOUNCE_NS = 50000000ULL;

    static constexpr int8_t TRANS_[16] = {
         0, -1,  1,  0,  1,  0,  0, -1,
        -1,  0,  0,  1,  0,  1, -1,  0
    };

#ifndef USE_BCM2835
    // lgpio interrupt-driven callbacks
    static void rotaryCB(int num_alerts, lgGpioAlert_p alerts, void* user) {
        auto* self = static_cast<RotaryEncoder*>(user);
        for (int i = 0; i < num_alerts; i++) self->onRotaryEdge();
    }

    static void buttonCB(int num_alerts, lgGpioAlert_p alerts, void* user) {
        auto* self = static_cast<RotaryEncoder*>(user);
        for (int i = 0; i < num_alerts; i++) {
            if (alerts[i].report.level == 0) {
                uint64_t ts = alerts[i].report.timestamp;
                if ((ts - self->btn_last_ts_) > BTN_DEBOUNCE_NS) {
                    self->btn_last_ts_ = ts;
                    self->button_pressed_.store(true, std::memory_order_relaxed);
                }
            }
        }
    }
#else
    // bcm2835 polling thread
    std::thread poll_thread_;
    std::atomic<bool> poll_running_{false};
    bool btn_prev_ = false;
    uint64_t btn_last_us_ = 0;
    static constexpr uint64_t BTN_DEBOUNCE_US = 50000;

    void pollLoop() {
        while (poll_running_) {
            onRotaryEdge();

            bool btn_cur = (hw->digitalRead(sw_pin_) == 0);
            if (btn_cur && !btn_prev_) {
                uint64_t now = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                if ((now - btn_last_us_) > BTN_DEBOUNCE_US) {
                    btn_last_us_ = now;
                    button_pressed_.store(true, std::memory_order_relaxed);
                }
            }
            btn_prev_ = btn_cur;

            std::this_thread::sleep_for(std::chrono::microseconds(1000)); // ~1kHz
        }
    }
#endif

    void onRotaryEdge() {
        uint8_t cur = static_cast<uint8_t>(
            (hw->digitalRead(clk_pin_) << 1) | hw->digitalRead(dt_pin_));
        int8_t step = TRANS_[(state_ << 2) | cur];
        state_ = cur;
        accumulator_ += step;

        if (accumulator_ >= detent_threshold_) {
            accumulator_ -= detent_threshold_;
            rotation_.fetch_add(1, std::memory_order_relaxed);
        } else if (accumulator_ <= -detent_threshold_) {
            accumulator_ += detent_threshold_;
            rotation_.fetch_add(-1, std::memory_order_relaxed);
        }
    }

public:
    RotaryEncoder(int clk, int dt, int sw, int detent_threshold = 4)
        : clk_pin_(clk), dt_pin_(dt), sw_pin_(sw),
          detent_threshold_(detent_threshold) {}

    void init() {
#ifndef USE_BCM2835
        // lgpio: interrupt-driven
        int h = hw->gpioHandle();
        lgGpioClaimAlert(h, LG_SET_PULL_UP, LG_BOTH_EDGES, clk_pin_, -1);
        lgGpioClaimAlert(h, LG_SET_PULL_UP, LG_BOTH_EDGES, dt_pin_, -1);
        lgGpioClaimAlert(h, LG_SET_PULL_UP, LG_BOTH_EDGES, sw_pin_, -1);
        lgGpioSetDebounce(h, clk_pin_, 5);
        lgGpioSetDebounce(h, dt_pin_, 5);
        lgGpioSetDebounce(h, sw_pin_, 5000);

        state_ = static_cast<uint8_t>(
            (lgGpioRead(h, clk_pin_) << 1) | lgGpioRead(h, dt_pin_));

        lgGpioSetAlertsFunc(h, clk_pin_, rotaryCB, this);
        lgGpioSetAlertsFunc(h, dt_pin_, rotaryCB, this);
        lgGpioSetAlertsFunc(h, sw_pin_, buttonCB, this);
#else
        // bcm2835: dedicated 1kHz polling thread (matches original RPi3B code)
        hw->claimInputPullUp(clk_pin_);
        hw->claimInputPullUp(dt_pin_);
        hw->claimInputPullUp(sw_pin_);
        state_ = static_cast<uint8_t>(
            (hw->digitalRead(clk_pin_) << 1) | hw->digitalRead(dt_pin_));
        btn_prev_ = (hw->digitalRead(sw_pin_) == 0);

        poll_running_ = true;
        poll_thread_ = std::thread(&RotaryEncoder::pollLoop, this);
#endif
    }

    void stop() {
#ifdef USE_BCM2835
        if (poll_running_) {
            poll_running_ = false;
            if (poll_thread_.joinable()) poll_thread_.join();
        }
#endif
    }

    ~RotaryEncoder() {
#ifndef USE_BCM2835
        if (hw && hw->gpioHandle() >= 0) {
            int h = hw->gpioHandle();
            lgGpioSetAlertsFunc(h, clk_pin_, nullptr, nullptr);
            lgGpioSetAlertsFunc(h, dt_pin_, nullptr, nullptr);
            lgGpioSetAlertsFunc(h, sw_pin_, nullptr, nullptr);
        }
#else
        stop();
#endif
    }

    int pollRotation() { return rotation_.exchange(0, std::memory_order_relaxed); }
    bool pollButton() { return button_pressed_.exchange(false, std::memory_order_relaxed); }
};

constexpr int8_t RotaryEncoder::TRANS_[16];

// Global flag
std::atomic<bool> g_shutdown_req{false};

// --- Main Application ---
class VisualizerApp {
    std::unique_ptr<Display> d_left, d_right;
    std::unique_ptr<AudioProcessor> audio;
    std::unique_ptr<MPDClient> mpd;
    std::unique_ptr<BluetoothMonitor> bluetooth;
    FontManager font_mgr;
    ControlState state;
    std::vector<std::unique_ptr<Visualization>> vizs;
    RotaryEncoder rotary{Config::ROT_CLK, Config::ROT_DT, Config::ROT_SW};
    bool prev_viz_sw = false;

    void onBluetoothConnectionChange(bool connected) {
        if (connected) { printf("Bluetooth connected - pausing MPD\n"); mpd->playPause(); }
        else printf("Bluetooth disconnected\n");
    }

    int overlay_volume = -1;
    std::chrono::steady_clock::time_point overlay_ts;
    static constexpr int OVERLAY_DURATION_MS = 1500;

    void drawVolumeOverlay(Display* d) {
        int vol = overlay_volume;
        if (vol < 0) return;
        d->drawRect(14, 24, 100, 18, false);
        for (int py = 25; py < 41; py++) {
            int row = (py/8)*Config::SCREEN_WIDTH;
            uint8_t mask = ~(1 << (py%8));
            for (int px = 15; px < 113; px++) d->buffer[row+px] &= mask;
        }
        d->drawRect(14, 24, 100, 18, false);
        int bar_w = (vol*88)/100;
        d->drawRect(18, 28, 88, 8, false);
        if (bar_w > 0) d->drawRect(18, 28, bar_w, 8, true);
        char buf[8]; snprintf(buf, sizeof(buf), "%d%%", vol);
        int tw = font_mgr.isInitialized() ? font_mgr.getTextWidth(buf, FontManager::SMALL) : 0;
        d->drawText((128-tw)/2, 23, buf, FontManager::SMALL);
    }

    void blankRows(Display* d, int from, int to) {
        int first_full = (from + 7) / 8, last_full = to / 8;
        for (int p = first_full; p < last_full; p++) memset(&d->buffer[p*128], 0, 128);
        if (from/8 < first_full) {
            int p = from/8; uint8_t m = 0;
            for (int b = from%8; b < 8; b++) m |= (1<<b);
            for (int px = 0; px < 128; px++) d->buffer[p*128+px] &= ~m;
        }
        if (to%8 != 0 && last_full < 8) {
            int p = last_full; uint8_t m = 0;
            for (int b = 0; b < to%8; b++) m |= (1<<b);
            for (int px = 0; px < 128; px++) d->buffer[p*128+px] &= ~m;
        }
    }

    void animateCRTBoth(bool power_on) {
        const int total_ms = 400;
        uint8_t saved_left[Config::SPI_CHUNK_SIZE], saved_right[Config::SPI_CHUNK_SIZE];
        memcpy(saved_left, d_left->buffer, Config::SPI_CHUNK_SIZE);
        memcpy(saved_right, d_right->buffer, Config::SPI_CHUNK_SIZE);

        auto start = std::chrono::steady_clock::now();
        while (true) {
            auto now = std::chrono::steady_clock::now();
            int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count();
            if (elapsed >= total_ms) break;

            Display* screens[2] = { d_left.get(), d_right.get() };
            uint8_t* saved[2] = { saved_left, saved_right };

            for (int s = 0; s < 2; s++) {
                auto* d = screens[s];
                memcpy(d->buffer, saved[s], Config::SPI_CHUNK_SIZE);

                float phase_t = (float)elapsed / (float)total_ms;
                if (power_on) phase_t = 1.0f - phase_t;

                if (phase_t < 0.5f) {
                    float p = phase_t * 2.0f;
                    int spread = (int)(31.0f * (1.0f - p));
                    int vt = 32-spread, vb = 32+spread;
                    if (vt > 0) blankRows(d, 0, vt);
                    if (vb < 63) blankRows(d, vb+1, 64);
                    for (int px = 0; px < 128; px++) { d->drawPixel(px, vt); d->drawPixel(px, vb); }
                } else {
                    float p = (phase_t - 0.5f) * 2.0f;
                    int hw_ = (int)(64.0f * (1.0f - p));
                    int xs = 64-hw_, xe = 64+hw_;
                    blankRows(d, 0, 32); blankRows(d, 33, 64);
                    uint8_t mask = (1 << 0);
                    for (int px = 0; px < xs; px++) d->buffer[4*128+px] &= ~mask;
                    for (int px = xe+1; px < 128; px++) d->buffer[4*128+px] &= ~mask;
                    for (int px = xs; px <= xe; px++) d->drawPixel(px, 32);
                }
                d->display();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        d_left->clear(); d_left->display();
        d_right->clear(); d_right->display();
    }

    public:
    VisualizerApp() {
        hw = createHardware();
        if (!hw->init()) throw std::runtime_error("Hardware Init Failed");

        d_left = std::make_unique<Display>(Config::LEFT_CS, Config::LEFT_DC, Config::LEFT_RST);
        d_right = std::make_unique<Display>(Config::RIGHT_CS, Config::RIGHT_DC, Config::RIGHT_RST);
        d_left->begin();
        d_right->begin();

        const char* fonts[] = {"trixel-square.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", nullptr};
        for (int i = 0; fonts[i]; i++) {
            if (font_mgr.init(fonts[i])) {
                d_left->setFont(&font_mgr); d_right->setFont(&font_mgr); break;
            }
        }

        mpd = std::make_unique<MPDClient>(); mpd->start();
        bluetooth = std::make_unique<BluetoothMonitor>();
        bluetooth->start([this](bool c) { onBluetoothConnectionChange(c); });
        audio = std::make_unique<AudioProcessor>();

        vizs.push_back(std::make_unique<VUMeterViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<SpectrumViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<EmptySpectrumViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<WaveformViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<TrackInfoViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<PlaylistEditorViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));

        setupGPIO();
    }

    ~VisualizerApp() {
        printf("Shutting down...\n");
        audio->stop(); bluetooth->stop(); mpd->stop();
        rotary.stop();
        d_left->shutdown(); d_right->shutdown();
        hw->digitalWrite(Config::POWER_LED, 0);
        hw->cleanup();
        printf("Displays powered off\n");
    }

    void setupGPIO() {
        uint8_t ins[] = { Config::VIZ_SW, Config::PLAY_SW, Config::FW_SW, Config::RW_SW };
        for (auto p : ins) hw->claimInputPullUp(p);
        hw->claimOutput(Config::POWER_LED, 1);
        rotary.init();
    }

    void pollControls() {
        int rot = rotary.pollRotation();
        bool rot_btn = rotary.pollButton();

        bool viz_raw = (hw->digitalRead(Config::VIZ_SW) == 0);
        bool viz_pressed = viz_raw && !prev_viz_sw;
        prev_viz_sw = viz_raw;

        bool any_button = viz_pressed || rot_btn || (rot != 0)
                       || (hw->digitalRead(Config::PLAY_SW) == 0)
                       || (hw->digitalRead(Config::FW_SW) == 0)
                       || (hw->digitalRead(Config::RW_SW) == 0);
        if (any_button) audio->resetSleepTimer();

        auto* editor = dynamic_cast<PlaylistEditorViz*>(vizs[state.current_viz].get());

        if (editor) {
            bool want_exit = false;
            for (int i = 0; i < abs(rot); i++)
                want_exit |= editor->handleInput(rot<0, rot>0, false, false);
            if (rot_btn || viz_pressed)
                want_exit |= editor->handleInput(false, false, rot_btn, viz_pressed);

            if (want_exit) {
                int n = (int)vizs.size();
                state.current_viz = (state.current_viz+1) % n;
                if (dynamic_cast<PlaylistEditorViz*>(vizs[state.current_viz].get()))
                    state.current_viz = (state.current_viz+1) % n;
                printf("Sortie editeur -> %s\n", vizs[state.current_viz]->getName());
            }
        } else {
            if (viz_pressed) {
                int n = (int)vizs.size();
                state.current_viz = (state.current_viz+1) % n;
                if (dynamic_cast<PlaylistEditorViz*>(vizs[state.current_viz].get()))
                    state.current_viz = (state.current_viz+1) % n;
                printf("Viz: %s\n", vizs[state.current_viz]->getName());
            }
            if (rot != 0) {
                mpd->setVolume(rot * 2);
                overlay_volume = mpd->getVolume();
                overlay_ts = std::chrono::steady_clock::now();
            }
            if (rot_btn) {
                for (int i = 0; i < (int)vizs.size(); i++) {
                    if (auto* ed = dynamic_cast<PlaylistEditorViz*>(vizs[i].get())) {
                        state.current_viz = i; ed->onActivate();
                        printf("Ouverture editeur (ROT_SW)\n"); break;
                    }
                }
            }
            if (hw->digitalRead(Config::PLAY_SW) == 0) { mpd->playPause(); hw->delay(200); }
            if (hw->digitalRead(Config::FW_SW) == 0)   { mpd->nextTrack(); hw->delay(200); }
            if (hw->digitalRead(Config::RW_SW) == 0)    { mpd->prevTrack(); hw->delay(200); }
        }
    }

    void run() {
        if (!audio->start()) return;
        using namespace std::chrono;
        auto next_frame = steady_clock::now();
        auto last_user_input = steady_clock::now();
        uint32_t auto_cycle_frames = 0;
        constexpr uint32_t CYCLE_INTERVAL = 36000;

        while (state.running && !g_shutdown_req) {
            pollControls();
            static int prev_viz = state.current_viz;
            if (state.current_viz != prev_viz) {
                last_user_input = steady_clock::now();
                auto_cycle_frames = 0; prev_viz = state.current_viz;
            }
            bool has_audio = audio->checkForAudio();

            if (!has_audio && !state.is_sleeping) {
                state.is_sleeping = true; audio->setSleepState(true);
                animateCRTBoth(false);
                d_left->sleep(); d_right->sleep();
                hw->digitalWrite(Config::POWER_LED, 0);
            } else if (has_audio && state.is_sleeping) {
                state.is_sleeping = false; audio->setSleepState(false);
                d_left->wake(); d_right->wake();
                hw->digitalWrite(Config::POWER_LED, 1);
                vizs[state.current_viz]->render(state, *audio);
                animateCRTBoth(true);
            }

            if (!state.is_sleeping) {
                bool in_editor = dynamic_cast<PlaylistEditorViz*>(vizs[state.current_viz].get()) != nullptr;

                if (!in_editor) {
                    auto since = duration_cast<seconds>(steady_clock::now()-last_user_input).count();
                    if (since > 30) {
                        auto_cycle_frames++;
                        if (auto_cycle_frames >= CYCLE_INTERVAL) {
                            int n = (int)vizs.size();
                            state.current_viz = (state.current_viz+1) % n;
                            if (dynamic_cast<PlaylistEditorViz*>(vizs[state.current_viz].get()))
                                state.current_viz = (state.current_viz+1) % n;
                            printf("Auto-cycle to: %s\n", vizs[state.current_viz]->getName());
                            auto_cycle_frames = 0;
                        }
                    }
                } else auto_cycle_frames = 0;

                vizs[state.current_viz]->render(state, *audio);

                if (!in_editor && overlay_volume >= 0) {
                    auto ems = duration_cast<milliseconds>(steady_clock::now()-overlay_ts).count();
                    if (ems < OVERLAY_DURATION_MS) {
                        drawVolumeOverlay(d_left.get()); drawVolumeOverlay(d_right.get());
                    } else overlay_volume = -1;
                }

                d_left->display(); d_right->display();
                auto now = steady_clock::now();
                if (now > next_frame) next_frame = now;
                next_frame += milliseconds(16);
                std::this_thread::sleep_until(next_frame);
            } else {
                std::this_thread::sleep_for(milliseconds(100));
            }
        }
    }
};

void sig_handler(int) { g_shutdown_req = true; }

int main() {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    try {
        VisualizerApp app;
        app.run();
    } catch (std::exception& e) {
        printf("Error: %s\n", e.what());
        if (hw) hw->cleanup();
        return 1;
    }

    printf("Clean exit.\n");
    return 0;
}