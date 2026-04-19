/*
Dual SSD1309 OLED Audio Visualizer for Raspberry Pi 5
Optimized C++ implementation using lgpio library


HiFiBerry DAC+, DAC+ ADC, DAC+ ADC Pro, DAC2 and Amp+, Studio DAC/ADC, Studio ADC
GPIO2-3 (pins 3 and 5) are used by our products for configuration. If you are experienced with I2C, you might add other slave devices. If you a a novice, we don’t recommend this at all.
GPIOs 18-21 (pins 12, 35, 38 and 40) are used for the sound interface. You can’t use them for any other purpose.


g++ -o visualizer visualizer.cpp \
    -O3 -march=native -std=c++17 \
    -I/usr/include/freetype2 \
    $(pkg-config --cflags --libs dbus-1) \
    -fstack-protector-strong -D_FORTIFY_SOURCE=2 -fPIE -pie -Wl,-z,relro,-z,now \
    -llgpio -lpthread -lasound -lfftw3f -lm -lfreetype -lmpdclient 
*/

#include <lgpio.h>
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
#include FT_FREETYPE_H

// --- Configuration Constants ---
namespace Config {
    constexpr int SCREEN_WIDTH = 128;
    constexpr int SCREEN_HEIGHT = 128;
    constexpr int SPI_CHUNK_SIZE = 4096; 
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
    constexpr uint8_t POWER_SW = 13;
    constexpr int GPIO_CHIP = 4;        // RPi5 uses gpiochip4 for user GPIO
    constexpr int SPI_DEVICE = 0;       // /dev/spidev0.x
    constexpr int SPI_CHANNEL = 0;      // CS0 (we manage CS manually)
    constexpr int SPI_SPEED = 20000000;  // 20MHz
}

namespace LGpio {
    int gpio_handle = -1;
    int spi_handle_cs0 = -1;
    int spi_handle_cs1 = -1;
    
    static constexpr int SPI_MODE_0 = 0;
    static constexpr int SPI_MODE_3 = 3;
    
    bool init(int spi_mode = SPI_MODE_0) {
        gpio_handle = lgGpiochipOpen(Config::GPIO_CHIP);
        if (gpio_handle < 0) {
            gpio_handle = lgGpiochipOpen(0);
            if (gpio_handle < 0) {
                printf("Failed to open GPIO chip: %s\n", lguErrorText(gpio_handle));
                return false;
            }
            printf("Using gpiochip0 (fallback)\n");
        } else {
            printf("Using gpiochip4 (RPi5)\n");
        }
        
        int spiFlags = spi_mode;
        
        spi_handle_cs0 = lgSpiOpen(Config::SPI_DEVICE, 0, Config::SPI_SPEED, spiFlags);
        if (spi_handle_cs0 < 0) {
            printf("Failed to open SPI CS0: %s\n", lguErrorText(spi_handle_cs0));
            lgGpiochipClose(gpio_handle);
            gpio_handle = -1;
            return false;
        }
        
        spi_handle_cs1 = lgSpiOpen(Config::SPI_DEVICE, 1, Config::SPI_SPEED, spiFlags);
        if (spi_handle_cs1 < 0) {
            printf("Failed to open SPI CS1: %s\n", lguErrorText(spi_handle_cs1));
            lgSpiClose(spi_handle_cs0);
            lgGpiochipClose(gpio_handle);
            gpio_handle = -1;
            spi_handle_cs0 = -1;
            return false;
        }
        
        printf("SPI initialized: CS0 and CS1 at %d Hz, Mode %d, Chunk size %d\n", 
               Config::SPI_SPEED, spi_mode, Config::SPI_CHUNK_SIZE);
        return true;
    }
    
    void cleanup() {
        if (spi_handle_cs0 >= 0) { lgSpiClose(spi_handle_cs0); spi_handle_cs0 = -1; }
        if (spi_handle_cs1 >= 0) { lgSpiClose(spi_handle_cs1); spi_handle_cs1 = -1; }
        if (gpio_handle >= 0) { lgGpiochipClose(gpio_handle); gpio_handle = -1; }
    }
    
    inline int claimOutput(int pin, int initialValue = 0) {
        return lgGpioClaimOutput(gpio_handle, 0, pin, initialValue);
    }
    
    inline int claimInput(int pin) {
        return lgGpioClaimInput(gpio_handle, 0, pin);
    }
    
    inline int claimInputPullUp(int pin) {
        return lgGpioClaimInput(gpio_handle, LG_SET_PULL_UP, pin);
    }
    
    inline void digitalWrite(int pin, int value) {
        lgGpioWrite(gpio_handle, pin, value);
    }
    
    inline int digitalRead(int pin) {
        return lgGpioRead(gpio_handle, pin);
    }
    
    inline void delay(int ms) {
        lguSleep(ms / 1000.0);
    }
    
    inline void delayMicroseconds(int us) {
        lguSleep(us / 1000000.0);
    }
    
    inline int getSpiHandle(int cs_pin) {
        return (cs_pin == Config::LEFT_CS) ? spi_handle_cs0 : spi_handle_cs1;
    }
    
    // SPI transfer single byte
    inline uint8_t spiTransfer(int cs_pin, uint8_t data) {
        char tx = static_cast<char>(data);
        char rx = 0;
        int handle = getSpiHandle(cs_pin);
        int result = lgSpiXfer(handle, &tx, &rx, 1);
        if (result < 0) {
            printf("SPI transfer error: %s\n", lguErrorText(result));
        }
        return static_cast<uint8_t>(rx);
    }
    
    // SPI write with chunking
    inline int spiWrite(int cs_pin, const uint8_t* buffer, size_t len) {
        int handle = getSpiHandle(cs_pin);
        size_t offset = 0;
        
        while (offset < len) {
            size_t chunk = std::min((size_t)Config::SPI_CHUNK_SIZE, len - offset);
            int result = lgSpiWrite(handle, 
                                    reinterpret_cast<const char*>(buffer + offset), 
                                    chunk);
            if (result < 0) {
                printf("SPI write error at %zu: %s\n", offset, lguErrorText(result));
                return result;
            }
            offset += chunk;
        }
        return (int)len;
    }
}

// Forward declarations
class Display;
class AudioProcessor;
struct ControlState;

// --- Simplified MPD Client ---
class MPDClient {
    private:
        struct mpd_connection* conn;
        std::thread mpd_thread;
        std::atomic<bool> thread_running;
        std::mutex data_mutex;
        
        std::string formatted_text;
        std::string host;
        int port;
        
        // Detailed track info
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
        
        // Timer local pour mise à jour du temps (approche ncmpcpp)
        std::chrono::steady_clock::time_point last_status_update;
        std::chrono::steady_clock::time_point last_bitrate_check;
        
        void updateFormattedText() {
            std::lock_guard<std::mutex> lock(data_mutex);
            
            if (!conn || mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) {
                formatted_text = "Waiting for MPD...";
                // Clear detailed info
                track_number = "";
                title = "No song playing";
                artist = "";
                album = "";
                date = "";
                bitrate_str = "";
                format_str = "";
                elapsed_time = 0;
                total_time = 0;
                is_playing = false;
                return;
            }
            
            struct mpd_song* song = mpd_run_current_song(conn);
            if (!song) {
                formatted_text = "No song playing";
                // Clear detailed info
                track_number = "";
                title = "No song playing";
                artist = "";
                album = "";
                date = "";
                bitrate_str = "";
                format_str = "";
                elapsed_time = 0;
                total_time = 0;
                is_playing = false;
                mpd_response_finish(conn);
                return;
            }
            
            // Extract song metadata
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
            
            // Build formatted text for scrolling
            std::stringstream ss;
            ss << std::setfill('0') << std::setw(2) << track << ". ";
            ss << (title_tag ? title_tag : "Unknown Title");
            if (artist_tag) ss << " - " << artist_tag;
            if (date_tag && strlen(date_tag) >= 4) ss << " (" << std::string(date_tag).substr(0, 4) << ")";
            
            formatted_text = ss.str();
            mpd_song_free(song);
            mpd_response_finish(conn);
            
            // Get playback status (bitrate, format, time, playing state)
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
                        << (int)format->bits << "bit/"      // Cast en int
                        << (int)format->channels << "ch";   // Cast en int
                    format_str = fmt.str();
                } else {
                    format_str = "N/A";
                }
                
                mpd_status_free(status);
                last_status_update = std::chrono::steady_clock::now();
            } else {
                bitrate_str = "N/A";
                format_str = "N/A";
                is_playing = false;
            }
            mpd_response_finish(conn);
        }
        
        void refreshBitrateOnly() {
            // Ne pas verrouiller ici, sera appelé depuis le thread MPD uniquement
            if (!conn || mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) {
                return;
            }
            
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
                // Connect if not connected
                if (!conn) {
                    conn = mpd_connection_new(host.c_str(), port, 5000);
                    if (!conn || mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) {
                        if (conn) {
                            mpd_connection_free(conn);
                            conn = nullptr;
                        }
                        // Sleep 5 seconds, but wake up instantly if we need to stop
                        for (int i = 0; i < 50; ++i) { 
                            if (!thread_running) return; 
                            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
                        }
                        continue;
                    }
                    printf("Connected to MPD at %s:%d\n", host.c_str(), port);
                    updateFormattedText();
                }
                
                // Enter idle mode
                if (!mpd_send_idle_mask(conn, MPD_IDLE_PLAYER)) {
                    printf("MPD idle failed, reconnecting...\n");
                    mpd_connection_free(conn);
                    conn = nullptr;
                    continue;
                }
                
                // Get the connection file descriptor
                int fd = mpd_connection_get_fd(conn);
                if (fd < 0) {
                    printf("Invalid MPD connection fd\n");
                    mpd_connection_free(conn);
                    conn = nullptr;
                    continue;
                }
                
                // Use select() with timeout to wait for events
                fd_set read_fds;
                struct timeval tv;
                
                while (thread_running) {
                    FD_ZERO(&read_fds);
                    FD_SET(fd, &read_fds);
                    
                    // Timeout de 100 milliseconde pour vérifier thread_running
                    tv.tv_sec = 0;
                    tv.tv_usec = 100000;
                    
                    int ret = select(fd + 1, &read_fds, nullptr, nullptr, &tv);
                    
                    if (ret < 0) {
                        // Erreur select
                        printf("Select error, reconnecting...\n");
                        mpd_connection_free(conn);
                        conn = nullptr;
                        break;
                    } else if (ret == 0) {
                        // Timeout toutes les 100ms - vérifier si rafraîchissement nécessaire
                        auto now = std::chrono::steady_clock::now();
                        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - last_bitrate_check).count();
                        
                        if (diff >= 1000) {  // Rafraîchir toutes les secondes
                            // Sortir proprement du idle mode avec noidle
                            mpd_run_noidle(conn);
                            
                            // Vérifier si la connexion est toujours valide
                            if (mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) {
                                printf("MPD connection error after noidle: %s\n", 
                                    mpd_connection_get_error_message(conn));
                                mpd_connection_free(conn);
                                conn = nullptr;
                                break;
                            }
                            
                            // Rafraîchir le bitrate
                            refreshBitrateOnly();
                            
                            // Réentrer en idle
                            break;
                        }
                        continue;
                    } else {
                        // Données disponibles
                        enum mpd_idle events = mpd_recv_idle(conn, false);
                        
                        if (mpd_connection_get_error(conn) != MPD_ERROR_SUCCESS) {
                            printf("MPD connection error: %s\n", 
                                mpd_connection_get_error_message(conn));
                            mpd_connection_free(conn);
                            conn = nullptr;
                            break;
                        }
                        
                        if (events & MPD_IDLE_PLAYER) {
                            updateFormattedText();
                        }
                        
                        // Sortir de la boucle select pour réentrer en idle
                        break;
                    }
                }
            }
            
            if (conn) {
                mpd_connection_free(conn);
                conn = nullptr;
            }
            printf("MPD thread stopped\n");
        }
        
    public:
        MPDClient(const std::string& mpd_host = "localhost", int mpd_port = 6600) 
            : conn(nullptr), thread_running(false), host(mpd_host), port(mpd_port) {
            formatted_text = "Waiting for MPD...";
            track_number = "";
            title = "No song playing";
            artist = "";
            album = "";
            date = "";
            bitrate_str = "";
            format_str = "";
            elapsed_time = 0;
            total_time = 0;
            is_playing = false;
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
            
            // Le thread se terminera au prochain timeout (max 100ms)
            if (mpd_thread.joinable()) {
                mpd_thread.join();
            }
        }
        
        std::string getFormattedText() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return formatted_text;
        }
        
        // Getters pour les infos détaillées
        std::string getTrackNumber() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return track_number;
        }
        
        std::string getTitle() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return title;
        }
        
        std::string getArtist() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return artist;
        }
        
        std::string getAlbum() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return album;
        }
        
        std::string getDate() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return date;
        }
        
        std::string getBitrate() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return bitrate_str;
        }
        
        std::string getFormat() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return format_str;
        }
        
        unsigned int getElapsedTime() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return elapsed_time;
        }
        
        unsigned int getTotalTime() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return total_time;
        }
        
        std::string formatTime(unsigned int seconds) {
            unsigned int mins = seconds / 60;
            unsigned int secs = seconds % 60;
            char buffer[16];
            snprintf(buffer, sizeof(buffer), "%02u:%02u", mins, secs);
            return std::string(buffer);
        }

        void playPause() {
            struct mpd_connection* ctrl = mpd_connection_new(host.c_str(), port, 1000);
            if (ctrl && mpd_connection_get_error(ctrl) == MPD_ERROR_SUCCESS) {
                mpd_run_toggle_pause(ctrl);
                mpd_connection_free(ctrl);
            }
        }
        
        void nextTrack() {
            struct mpd_connection* ctrl = mpd_connection_new(host.c_str(), port, 1000);
            if (ctrl && mpd_connection_get_error(ctrl) == MPD_ERROR_SUCCESS) {
                mpd_run_next(ctrl);
                mpd_connection_free(ctrl);
            }
        }
        
        void prevTrack() {
            struct mpd_connection* ctrl = mpd_connection_new(host.c_str(), port, 1000);
            if (ctrl && mpd_connection_get_error(ctrl) == MPD_ERROR_SUCCESS) {
                mpd_run_previous(ctrl);
                mpd_connection_free(ctrl);
            }
        }

        // Mise à jour locale du temps écoulé (approche ncmpcpp)
        void updateElapsedTime() {
            std::lock_guard<std::mutex> lock(data_mutex);
            
            // Si on joue, incrémenter localement le temps écoulé
            if (is_playing && total_time > 0) {
                auto now = std::chrono::steady_clock::now();
                auto diff = std::chrono::duration_cast<std::chrono::seconds>(
                    now - last_status_update).count();
                
                // Mettre à jour le temps local
                if (diff > 0) {
                    elapsed_time += diff;
                    if (elapsed_time > total_time) {
                        elapsed_time = total_time;
                    }
                    last_status_update = now;
                }
            }
        }
};

// --- Bluetooth Monitor ---
class BluetoothMonitor {
    private:
        DBusConnection* conn;
        std::thread monitor_thread;
        std::atomic<bool> running{false};
        std::mutex data_mutex;
        
        // Connection state
        std::atomic<bool> bt_connected{false};
        std::atomic<bool> bt_playing{false};
        
        // Track info from AVRCP
        std::string bt_title;
        std::string bt_artist;
        std::string bt_album;
        uint32_t bt_duration{0};
        uint32_t bt_position{0};
        
        // Callback for connection changes
        std::function<void(bool)> on_connection_change;
        
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
                                if (strcmp(interface, "org.bluez.MediaPlayer1") == 0) {
                                    parseMediaPlayer(msg);
                                }
                                else if (strcmp(interface, "org.bluez.MediaTransport1") == 0) {
                                    parseTransport(msg);
                                }
                            }
                        }
                    }
                    
                    dbus_message_unref(msg);
                }
            }
            
            std::chrono::steady_clock::time_point last_position_update;
            
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
                    }
                    else if (strcmp(key, "Track") == 0) {
                        parseTrack(&variant);
                    }
                    else if (strcmp(key, "Position") == 0) {
                        dbus_message_iter_get_basic(&variant, &bt_position);
                        // Mettre à jour le timestamp quand on reçoit une position de AVRCP
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
                    }
                    else if (strcmp(key, "Artist") == 0) {
                        dbus_message_iter_get_basic(&value, &str_val);
                        bt_artist = str_val ? str_val : "";
                    }
                    else if (strcmp(key, "Album") == 0) {
                        dbus_message_iter_get_basic(&value, &str_val);
                        bt_album = str_val ? str_val : "";
                    }
                    else if (strcmp(key, "Duration") == 0) {
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
                        
                        if (bt_connected != was_connected && on_connection_change) {
                            on_connection_change(bt_connected);
                        }
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
                dbus_error_free(&err);
                return false;
            }
            
            dbus_bus_add_match(conn,
                "type='signal',interface='org.freedesktop.DBus.Properties',member='PropertiesChanged'",
                &err);
            
            if (dbus_error_is_set(&err)) {
                printf("D-Bus match error: %s\n", err.message);
                dbus_error_free(&err);
                return false;
            }
            
            running = true;
            monitor_thread = std::thread(&BluetoothMonitor::monitorLoop, this);
            printf("Bluetooth monitor started\n");
            return true;
        }
        
        void stop() {
            running = false;
            if (monitor_thread.joinable()) {
                monitor_thread.join();
            }
            if (conn) {
                dbus_connection_unref(conn);
                conn = nullptr;
            }
        }
        
        bool isConnected() const { return bt_connected; }
        bool isPlaying() const { return bt_playing; }
        
        std::string getTitle() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return bt_title;
        }
        
        std::string getArtist() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return bt_artist;
        }
        
        std::string getAlbum() {
            std::lock_guard<std::mutex> lock(data_mutex);
            return bt_album;
        }
        
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
            
            // Si on joue, incrémenter localement la position
            if (bt_playing && bt_duration > 0) {
                auto now = std::chrono::steady_clock::now();
                auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_position_update).count();
                
                // Mettre à jour la position locale
                if (diff > 0) {
                    bt_position += diff;
                    if (bt_position > bt_duration) {
                        bt_position = bt_duration;
                    }
                    last_position_update = now;
                }
            }
        }
        
        uint32_t getDuration() const { return bt_duration; }
        uint32_t getPosition() const { return bt_position; }
};

// --- Font Manager (RGB Version - No Anti-aliasing) ---
class FontManager {
    private:
        FT_Library library;
        FT_Face face_regular, face_small, face_large;
        bool initialized;
        
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
            FT_Set_Pixel_Sizes(face_regular, 0, 12);
            
            if (!FT_New_Face(library, font_path, 0, &face_small)) FT_Set_Pixel_Sizes(face_small, 0, 8);
            if (!FT_New_Face(library, font_path, 0, &face_large)) FT_Set_Pixel_Sizes(face_large, 0, 16);
            
            initialized = true;
            return true;
        }
        
        bool renderText(const char* text, uint8_t* buffer, int buf_width, int buf_height, 
                        int x, int y, FontSize size = REGULAR, uint16_t color = 0xFFFF) {
            if (!initialized || !text) return false;
            
            FT_Face face = face_regular;
            if (size == SMALL && face_small) face = face_small;
            if (size == LARGE && face_large) face = face_large;
            
            int cursor_x = x;
            int baseline_y = y;
            
            const unsigned char* utext = (const unsigned char*)text;
            
            while (*utext) {
                unsigned int codepoint = 0;
                int bytes = 0;
                
                // UTF-8 decode
                if ((*utext & 0x80) == 0x00) {
                    codepoint = *utext;
                    bytes = 1;
                } else if ((*utext & 0xE0) == 0xC0) {
                    codepoint = (*utext & 0x1F) << 6;
                    codepoint |= (*(utext+1) & 0x3F);
                    bytes = 2;
                } else if ((*utext & 0xF0) == 0xE0) {
                    codepoint = (*utext & 0x0F) << 12;
                    codepoint |= (*(utext+1) & 0x3F) << 6;
                    codepoint |= (*(utext+2) & 0x3F);
                    bytes = 3;
                } else if ((*utext & 0xF8) == 0xF0) {
                    codepoint = (*utext & 0x07) << 18;
                    codepoint |= (*(utext+1) & 0x3F) << 12;
                    codepoint |= (*(utext+2) & 0x3F) << 6;
                    codepoint |= (*(utext+3) & 0x3F);
                    bytes = 4;
                } else {
                    utext++;
                    continue;
                }
                
                if (FT_Load_Char(face, codepoint, FT_LOAD_RENDER)) { 
                    utext += bytes; 
                    continue; 
                }
                
                FT_GlyphSlot slot = face->glyph;
                FT_Bitmap* bitmap = &slot->bitmap;
                
                int start_x = cursor_x + slot->bitmap_left;
                int start_y = baseline_y - slot->bitmap_top;
                
                uint8_t hi = color >> 8;
                uint8_t lo = color & 0xFF;
                
                for (unsigned int row = 0; row < bitmap->rows; row++) {
                    for (unsigned int col = 0; col < bitmap->width; col++) {
                        int px = start_x + col;
                        int py = start_y + row;
                        
                        if (px >= 0 && px < buf_width && py >= 0 && py < buf_height) {
                            uint8_t alpha = bitmap->buffer[row * bitmap->pitch + col];
                            
                            // Simple threshold - no anti-aliasing
                            if (alpha > 127) {
                                size_t idx = (py * buf_width + px) * 2;
                                buffer[idx] = hi;
                                buffer[idx + 1] = lo;
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
            if (size == LARGE && face_large) face = face_large;
            
            int width = 0;
            const unsigned char* utext = (const unsigned char*)text;
            
            while (*utext) {
                unsigned int codepoint = 0;
                int bytes = 0;
                
                if ((*utext & 0x80) == 0x00) {
                    codepoint = *utext;
                    bytes = 1;
                } else if ((*utext & 0xE0) == 0xC0) {
                    codepoint = (*utext & 0x1F) << 6;
                    codepoint |= (*(utext+1) & 0x3F);
                    bytes = 2;
                } else if ((*utext & 0xF0) == 0xE0) {
                    codepoint = (*utext & 0x0F) << 12;
                    codepoint |= (*(utext+1) & 0x3F) << 6;
                    codepoint |= (*(utext+2) & 0x3F);
                    bytes = 3;
                } else if ((*utext & 0xF8) == 0xF0) {
                    codepoint = (*utext & 0x07) << 18;
                    codepoint |= (*(utext+1) & 0x3F) << 12;
                    codepoint |= (*(utext+2) & 0x3F) << 6;
                    codepoint |= (*(utext+3) & 0x3F);
                    bytes = 4;
                } else {
                    utext++;
                    continue;
                }
                
                if (!FT_Load_Char(face, codepoint, FT_LOAD_DEFAULT)) {
                    width += face->glyph->advance.x >> 6;
                }
                utext += bytes;
            }
            return width;
        }
        
        bool isInitialized() const { return initialized; }
};

class Display {
    private:
        uint8_t _cs, _dc, _rst;
        FontManager* font_manager;

        static constexpr int WIDTH = Config::SCREEN_WIDTH;
        static constexpr int HEIGHT = Config::SCREEN_HEIGHT;
        static constexpr size_t BUFFER_SIZE = WIDTH * HEIGHT * 2;

    public:
    uint8_t buffer[BUFFER_SIZE];
    
    Display(uint8_t cs, uint8_t dc, uint8_t rst) 
        : _cs(cs), _dc(dc), _rst(rst), font_manager(nullptr) {
        memset(buffer, 0x00, sizeof(buffer));
    }
    
    bool begin() {
        // Setup DC and RST as outputs
        int result = LGpio::claimOutput(_dc, 0);
        if (result < 0) {
            printf("Failed to claim DC pin %d: %s\n", _dc, lguErrorText(result));
            return false;
        }
        
        result = LGpio::claimOutput(_rst, 1);
        if (result < 0) {
            printf("Failed to claim RST pin %d: %s\n", _rst, lguErrorText(result));
            return false;
        }
        
        // Hardware reset with proper timing
        printf("Resetting display...\n");
        LGpio::digitalWrite(_rst, 1);
        LGpio::delay(100);
        LGpio::digitalWrite(_rst, 0);
        LGpio::delay(100);
        LGpio::digitalWrite(_rst, 1);
        LGpio::delay(100);
        
        // SSD1351 initialization sequence (verified from Linux fbtft)
        printf("Initializing SSD1351...\n");
        
        // Unlock commands
        sendCommand(0xFD); sendData(0x12);
        sendCommand(0xFD); sendData(0xB1);
        
        // Display OFF during init
        sendCommand(0xAE);
        
        // Clock divider / oscillator frequency
        sendCommand(0xB3); sendData(0xF1);
        
        // Mux ratio (128 lines)
        sendCommand(0xCA); sendData(0x7F);
        
        // Set remap & color depth
        // 0x74 = 0b01110100:
        //   Bit 0: 0 = Horizontal address increment
        //   Bit 1: 0 = Column 0 mapped to SEG0
        //   Bit 2: 1 = Color sequence C->B->A (RGB order)
        //   Bit 4: 1 = Scan from COM[N-1] to COM0
        //   Bit 5: 1 = Enable COM split odd/even
        //   Bit 6: 1 = 65k color format (RGB565)
        sendCommand(0xA0); sendData(0x74);
        
        // Column address range
        sendCommand(0x15); sendData(0x00); sendData(0x7F);
        
        // Row address range
        sendCommand(0x75); sendData(0x00); sendData(0x7F);
        
        // Display start line
        sendCommand(0xA1); sendData(0x00);
        
        // Display offset
        sendCommand(0xA2); sendData(0x00);
        
        // GPIO (both disabled)
        sendCommand(0xB5); sendData(0x00);
        
        // Function selection (internal VDD regulator)
        sendCommand(0xAB); sendData(0x01);
        
        // Precharge period
        sendCommand(0xB1); sendData(0x32);
        
        // Set segment low voltage (VSL)
        sendCommand(0xB4); 
        sendData(0xA0);  // External VSL
        sendData(0xB5);  
        sendData(0x55);
        
        // Precharge voltage level
        sendCommand(0xBB); sendData(0x17);
        
        // VCOMH voltage
        sendCommand(0xBE); sendData(0x05);
        
        // Contrast for R, G, B
        sendCommand(0xC1); 
        sendData(0xC8);  // Red
        sendData(0x80);  // Green
        sendData(0xC8);  // Blue
        
        // Master contrast
        sendCommand(0xC7); sendData(0x0F);
        
        // Second precharge period
        sendCommand(0xB6); sendData(0x01);
        
        // Normal display mode (not inverted, not all-on, not all-off)
        sendCommand(0xA6);
        
        LGpio::delay(100);
        
        // Display ON
        sendCommand(0xAF);
        
        printf("SSD1351 initialization complete\n");
        
        // Clear to black
        clear();
        display();
        
        return true;
    }
    // Call this once in begin() after init:
    void setFullWindow() {
        sendCommand(0x15); sendData(0x00); sendData(0x7F);
        sendCommand(0x75); sendData(0x00); sendData(0x7F);
    }
    
    void sendCommand(uint8_t cmd) {
        LGpio::digitalWrite(_dc, 0);  // DC LOW = Command
        LGpio::delayMicroseconds(1);  // Small delay for DC to settle
        LGpio::spiTransfer(_cs, cmd);
    }
    
    void sendData(uint8_t data) {
        LGpio::digitalWrite(_dc, 1);  // DC HIGH = Data
        LGpio::delayMicroseconds(1);  // Small delay for DC to settle
        LGpio::spiTransfer(_cs, data);
    }
    
    void sendDataBuffer(const uint8_t* data, size_t len) {
        LGpio::digitalWrite(_dc, 1);  // DC HIGH = Data
        LGpio::delayMicroseconds(1);
        LGpio::spiWrite(_cs, data, len);
    }
    
    void setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
        sendCommand(0x15);  // Column address
        sendData(x0);
        sendData(x1);
        
        sendCommand(0x75);  // Row address
        sendData(y0);
        sendData(y1);
        
        sendCommand(0x5C);  // Write RAM command
    }
    
    void display() {
        sendCommand(0x5C);  // Write RAM command only
        sendDataBuffer(buffer, sizeof(buffer));
    }
    
    void clear() { 
        memset(buffer, 0x00, sizeof(buffer)); 
    }
    
    void fill(uint16_t color) {
        uint8_t hi = color >> 8;
        uint8_t lo = color & 0xFF;
        for (size_t i = 0; i < BUFFER_SIZE; i += 2) {
            buffer[i + 1] = lo;
            buffer[i] = hi;
        }
    }
    
    void sleep() { sendCommand(0xAE); }
    void wake()  { sendCommand(0xAF); }
    
    void shutdown() {
        sendCommand(0xAE);
        clear();
        display();
    }
    
    static uint16_t color(uint8_t r, uint8_t g, uint8_t b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }
    
    void drawPixel(int x, int y, uint16_t col = 0xFFFF) {
        if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
        size_t idx = (y * WIDTH + x) * 2;
        buffer[idx] = col >> 8;
        buffer[idx + 1] = col & 0xFF;

    }
    
    void drawLine(int x0, int y0, int x1, int y1, uint16_t color = 0xFFFF) {
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while (true) {
            drawPixel(x0, y0, color);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx)  { err += dx; y0 += sy; }
        }
    }
    
    void drawRect(int x, int y, int w, int h,  bool filled = false, uint16_t color = 0xFFFF) {
        if (filled) {
            for (int py = std::max(0, y); py < std::min(HEIGHT, y + h); py++) {
                for (int px = std::max(0, x); px < std::min(WIDTH, x + w); px++) {
                    drawPixel(px, py, color);
                }
            }
        } else {
            drawLine(x, y, x + w - 1, y, color);
            drawLine(x + w - 1, y, x + w - 1, y + h - 1, color);
            drawLine(x + w - 1, y + h - 1, x, y + h - 1, color);
            drawLine(x, y + h - 1, x, y, color);
        }
    }

    void drawCircle(int x0, int y0, int radius, bool filled = false, uint16_t color = 0xFFFF) {
        int x = radius, y = 0, err = 0;
        while (x >= y) {
            if (filled) {
                drawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
                drawLine(x0 - x, y0 - y, x0 + x, y0 - y, color);
                drawLine(x0 - y, y0 + x, x0 + y, y0 + x, color);
                drawLine(x0 - y, y0 - x, x0 + y, y0 - x, color);
            } else {
                drawPixel(x0 + x, y0 + y, color); drawPixel(x0 + y, y0 + x, color);
                drawPixel(x0 - y, y0 + x, color); drawPixel(x0 - x, y0 + y, color);
                drawPixel(x0 - x, y0 - y, color); drawPixel(x0 - y, y0 - x, color);
                drawPixel(x0 + y, y0 - x, color); drawPixel(x0 + x, y0 - y, color);
            }
            if (err <= 0) { y += 1; err += 2 * y + 1; }
            if (err > 0)  { x -= 1; err -= 2 * x + 1; }
        }
    }

    void setFont(FontManager* fm) { font_manager = fm; }
    
    void drawText(uint8_t x, uint8_t y, const char* text, 
                  uint16_t color = 0xFFFF,
                  FontManager::FontSize size = FontManager::SMALL) {
        if (font_manager) {
            // Note: FontManager will need adaptation for RGB buffer
            font_manager->renderText(text, buffer, WIDTH, HEIGHT, x, y, size, color);
        }
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
        
        struct FreqBand {
            int low, high;
            float correction;
        };
        
        static constexpr FreqBand FREQ_BANDS[7] = {
            {63, 120, 0.7f}, {120, 350, 1.2f}, {350, 900, 2.0f},
            {900, 2000, 6.0f}, {2000, 5000, 11.0f},
            {5000, 10000, 30.0f}, {10000, 16000, 50.0f}
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
        
        std::array<float, 7> prev_left_spectrum{};
        std::array<float, 7> prev_right_spectrum{};
        
        float noise_reduction = 77.0f;
        float sensitivity = 100.0f;
        float integral_factor, gravity_factor, scale_factor;
        
        // Sleep detection
        std::chrono::steady_clock::time_point last_audio_time;
        std::atomic<float> max_amplitude{0.0f};
        
        void createHannWindow(float* window, int size) {
            for (int i = 0; i < size; i++) {
                window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (size - 1)));
            }
        }
        
        void audioThreadFunc() {
            int16_t* audio_buffer = new int16_t[FRAMES_PER_BUFFER * CHANNELS];
            
            while (thread_running) {
                // Use smaller buffer during sleep for faster wake detection
                int frames_to_read = is_sleeping ? 256 : FRAMES_PER_BUFFER;
                
                int frames = snd_pcm_readi(pcm_handle, audio_buffer, frames_to_read);
                if (frames < 0) {
                    frames = snd_pcm_recover(pcm_handle, frames, 0);
                    if (frames < 0) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
                        continue;
                    }
                }
                
                float frame_max = 0.0f;
                
                // During sleep, only calculate max amplitude (skip buffer updates)
                if (is_sleeping) {
                    for (int i = 0; i < frames * CHANNELS; i++) {
                        frame_max = std::max(frame_max, std::abs(audio_buffer[i] / 32768.0f));
                    }
                    max_amplitude = frame_max;
                    if (frame_max > SILENCE_THRESHOLD) {
                        last_audio_time = std::chrono::steady_clock::now();
                    }
                    continue;  // Skip buffer writes during sleep
                }
                
                // Normal processing when awake
                std::lock_guard<std::mutex> lock(buffer_mutex);
                size_t pos = write_pos;
                
                for (int i = 0; i < frames; i++) {
                    circular_buffer_left[pos] = audio_buffer[i * CHANNELS] / 32768.0f;
                    circular_buffer_right[pos] = (CHANNELS > 1) ? 
                        audio_buffer[i * CHANNELS + 1] / 32768.0f : circular_buffer_left[pos];
                    
                    frame_max = std::max(frame_max, std::abs(circular_buffer_left[pos]));
                    frame_max = std::max(frame_max, std::abs(circular_buffer_right[pos]));
                    
                    pos = (pos + 1) % (FFT_SIZE_BASS * 2);
                }
                write_pos = pos;
                
                // Update max amplitude for sleep detection
                max_amplitude = frame_max;
                if (frame_max > SILENCE_THRESHOLD) {
                    last_audio_time = std::chrono::steady_clock::now();
                }
            }
            
            delete[] audio_buffer;
        }

        void updateParameters() {
            float nr_normalized = noise_reduction / 100.0f;
            integral_factor = nr_normalized * 0.95f;
            gravity_factor = 1.0f - (nr_normalized * 0.8f);
            gravity_factor = std::max(gravity_factor, 0.2f);
            scale_factor = (sensitivity / 100.0f) * 1.8f;
        }
    
    public:
    AudioProcessor() : pcm_handle(nullptr), thread_running(false) {
        int buffer_size = FFT_SIZE_BASS * 2;
        circular_buffer_left = new float[buffer_size]();
        circular_buffer_right = new float[buffer_size]();
        
        // Allocate FFT resources
        fft_in_bass = (float*)fftwf_malloc(sizeof(float) * FFT_SIZE_BASS);
        fft_out_bass = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * (FFT_SIZE_BASS/2 + 1));
        plan_bass = fftwf_plan_dft_r2c_1d(FFT_SIZE_BASS, fft_in_bass, fft_out_bass, FFTW_ESTIMATE);
        
        fft_in_mid = (float*)fftwf_malloc(sizeof(float) * FFT_SIZE_MID);
        fft_out_mid = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * (FFT_SIZE_MID/2 + 1));
        plan_mid = fftwf_plan_dft_r2c_1d(FFT_SIZE_MID, fft_in_mid, fft_out_mid, FFTW_ESTIMATE);
        
        fft_in_treble = (float*)fftwf_malloc(sizeof(float) * FFT_SIZE_TREBLE);
        fft_out_treble = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * (FFT_SIZE_TREBLE/2 + 1));
        plan_treble = fftwf_plan_dft_r2c_1d(FFT_SIZE_TREBLE, fft_in_treble, fft_out_treble, FFTW_ESTIMATE);
        
        // Create windows
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
        delete[] circular_buffer_left;
        delete[] circular_buffer_right;
        delete[] window_bass;
        delete[] window_mid;
        delete[] window_treble;
        
        fftwf_destroy_plan(plan_bass);
        fftwf_destroy_plan(plan_mid);
        fftwf_destroy_plan(plan_treble);
        fftwf_free(fft_in_bass);
        fftwf_free(fft_out_bass);
        fftwf_free(fft_in_mid);
        fftwf_free(fft_out_mid);
        fftwf_free(fft_in_treble);
        fftwf_free(fft_out_treble);
    }
    
    bool start() {
        int err = snd_pcm_open(&pcm_handle, "cava", SND_PCM_STREAM_CAPTURE, 0);
        if (err < 0) {
            err = snd_pcm_open(&pcm_handle, "hw:Loopback,1", SND_PCM_STREAM_CAPTURE, 0);
        }
        if (err < 0) return false;
        
        snd_pcm_hw_params_t* hw_params;
        snd_pcm_hw_params_alloca(&hw_params);
        snd_pcm_hw_params_any(pcm_handle, hw_params);
        snd_pcm_hw_params_set_access(pcm_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(pcm_handle, hw_params, SND_PCM_FORMAT_S16_LE);
        snd_pcm_hw_params_set_channels(pcm_handle, hw_params, CHANNELS);
        
        unsigned int rate = SAMPLE_RATE;
        snd_pcm_hw_params_set_rate_near(pcm_handle, hw_params, &rate, 0);
        
        if (snd_pcm_hw_params(pcm_handle, hw_params) < 0) {
            snd_pcm_close(pcm_handle);
            return false;
        }
        
        thread_running = true;
        audio_thread = std::thread(&AudioProcessor::audioThreadFunc, this);
        return true;
    }

    void setSleepState(bool sleeping) {
        is_sleeping = sleeping;
    }
    
    void stop() {
        if (thread_running) {
            thread_running = false;
            
            // FIX: Force ALSA to stop waiting for data immediately
            if (pcm_handle) {
                snd_pcm_drop(pcm_handle);
            }

            if (audio_thread.joinable()) audio_thread.join();
        }
        
        if (pcm_handle) {
            snd_pcm_close(pcm_handle);
            pcm_handle = nullptr;
        }
    }
    
    bool checkForAudio() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_audio_time).count();
        return elapsed < SLEEP_TIMEOUT_SEC;
    }
    
    void getSpectrumData(std::array<int, 7>& left_out, std::array<int, 7>& right_out) {
        std::array<float, 7> left_bands{}, right_bands{};
        
        // Get buffer data (need full BASS size for largest FFT)
        float* temp_left = new float[FFT_SIZE_BASS];
        float* temp_right = new float[FFT_SIZE_BASS];
        
        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            size_t read_pos = (write_pos + FFT_SIZE_BASS * 2 - FFT_SIZE_BASS) % (FFT_SIZE_BASS * 2);
            for (int i = 0; i < FFT_SIZE_BASS; i++) {
                temp_left[i] = circular_buffer_left[read_pos];
                temp_right[i] = circular_buffer_right[read_pos];
                read_pos = (read_pos + 1) % (FFT_SIZE_BASS * 2);
            }
        }
        
        // ===== BASS FFT (bands 0, 1, 2) - 63Hz to 900Hz =====
        for (int i = 0; i < FFT_SIZE_BASS; i++) {
            fft_in_bass[i] = temp_left[i] * window_bass[i];
        }
        fftwf_execute(plan_bass);
        
        for (int i = 0; i < 3; i++) {  // Only bands 0-2
            int low_idx = FREQ_BANDS[i].low * FFT_SIZE_BASS / SAMPLE_RATE;
            int high_idx = FREQ_BANDS[i].high * FFT_SIZE_BASS / SAMPLE_RATE;
            
            float sum = 0.0f;
            for (int j = low_idx; j < high_idx && j < FFT_SIZE_BASS/2; j++) {
                float mag = sqrtf(fft_out_bass[j][0] * fft_out_bass[j][0] + 
                                fft_out_bass[j][1] * fft_out_bass[j][1]);
                sum += mag * mag;
            }
            
            left_bands[i] = sqrtf(sum / (high_idx - low_idx)) * scale_factor * FREQ_BANDS[i].correction;
        }
        
        // ===== MID FFT (bands 3, 4) - 900Hz to 5000Hz =====
        for (int i = 0; i < FFT_SIZE_MID; i++) {
            fft_in_mid[i] = temp_left[i] * window_mid[i];
        }
        fftwf_execute(plan_mid);
        
        for (int i = 3; i < 5; i++) {  // Bands 3-4
            int low_idx = FREQ_BANDS[i].low * FFT_SIZE_MID / SAMPLE_RATE;
            int high_idx = FREQ_BANDS[i].high * FFT_SIZE_MID / SAMPLE_RATE;
            
            float sum = 0.0f;
            for (int j = low_idx; j < high_idx && j < FFT_SIZE_MID/2; j++) {
                float mag = sqrtf(fft_out_mid[j][0] * fft_out_mid[j][0] + 
                                fft_out_mid[j][1] * fft_out_mid[j][1]);
                sum += mag * mag;
            }
            
            left_bands[i] = sqrtf(sum / (high_idx - low_idx)) * scale_factor * FREQ_BANDS[i].correction;
        }
        
        // ===== TREBLE FFT (bands 5, 6) - 5000Hz to 16000Hz =====
        for (int i = 0; i < FFT_SIZE_TREBLE; i++) {
            fft_in_treble[i] = temp_left[i] * window_treble[i];
        }
        fftwf_execute(plan_treble);
        
        for (int i = 5; i < 7; i++) {  // Bands 5-6
            int low_idx = FREQ_BANDS[i].low * FFT_SIZE_TREBLE / SAMPLE_RATE;
            int high_idx = FREQ_BANDS[i].high * FFT_SIZE_TREBLE / SAMPLE_RATE;
            
            float sum = 0.0f;
            for (int j = low_idx; j < high_idx && j < FFT_SIZE_TREBLE/2; j++) {
                float mag = sqrtf(fft_out_treble[j][0] * fft_out_treble[j][0] + 
                                fft_out_treble[j][1] * fft_out_treble[j][1]);
                sum += mag * mag;
            }
            
            left_bands[i] = sqrtf(sum / (high_idx - low_idx)) * scale_factor * FREQ_BANDS[i].correction;
        }
        
        // ===== RIGHT CHANNEL - Same process =====
        
        // BASS FFT for right channel
        for (int i = 0; i < FFT_SIZE_BASS; i++) {
            fft_in_bass[i] = temp_right[i] * window_bass[i];
        }
        fftwf_execute(plan_bass);
        
        for (int i = 0; i < 3; i++) {
            int low_idx = FREQ_BANDS[i].low * FFT_SIZE_BASS / SAMPLE_RATE;
            int high_idx = FREQ_BANDS[i].high * FFT_SIZE_BASS / SAMPLE_RATE;
            
            float sum = 0.0f;
            for (int j = low_idx; j < high_idx && j < FFT_SIZE_BASS/2; j++) {
                float mag = sqrtf(fft_out_bass[j][0] * fft_out_bass[j][0] + 
                                fft_out_bass[j][1] * fft_out_bass[j][1]);
                sum += mag * mag;
            }
            
            right_bands[i] = sqrtf(sum / (high_idx - low_idx)) * scale_factor * FREQ_BANDS[i].correction;
        }
        
        // MID FFT for right channel
        for (int i = 0; i < FFT_SIZE_MID; i++) {
            fft_in_mid[i] = temp_right[i] * window_mid[i];
        }
        fftwf_execute(plan_mid);
        
        for (int i = 3; i < 5; i++) {
            int low_idx = FREQ_BANDS[i].low * FFT_SIZE_MID / SAMPLE_RATE;
            int high_idx = FREQ_BANDS[i].high * FFT_SIZE_MID / SAMPLE_RATE;
            
            float sum = 0.0f;
            for (int j = low_idx; j < high_idx && j < FFT_SIZE_MID/2; j++) {
                float mag = sqrtf(fft_out_mid[j][0] * fft_out_mid[j][0] + 
                                fft_out_mid[j][1] * fft_out_mid[j][1]);
                sum += mag * mag;
            }
            
            right_bands[i] = sqrtf(sum / (high_idx - low_idx)) * scale_factor * FREQ_BANDS[i].correction;
        }
        
        // TREBLE FFT for right channel
        for (int i = 0; i < FFT_SIZE_TREBLE; i++) {
            fft_in_treble[i] = temp_right[i] * window_treble[i];
        }
        fftwf_execute(plan_treble);
        
        for (int i = 5; i < 7; i++) {
            int low_idx = FREQ_BANDS[i].low * FFT_SIZE_TREBLE / SAMPLE_RATE;
            int high_idx = FREQ_BANDS[i].high * FFT_SIZE_TREBLE / SAMPLE_RATE;
            
            float sum = 0.0f;
            for (int j = low_idx; j < high_idx && j < FFT_SIZE_TREBLE/2; j++) {
                float mag = sqrtf(fft_out_treble[j][0] * fft_out_treble[j][0] + 
                                fft_out_treble[j][1] * fft_out_treble[j][1]);
                sum += mag * mag;
            }
            
            right_bands[i] = sqrtf(sum / (high_idx - low_idx)) * scale_factor * FREQ_BANDS[i].correction;
        }
        
        // ===== SMOOTHING - Apply temporal smoothing with gravity =====
        for (int i = 0; i < 7; i++) {
            float smoothed_left = integral_factor * prev_left_spectrum[i] + 
                                (1.0f - integral_factor) * left_bands[i];
            float smoothed_right = integral_factor * prev_right_spectrum[i] + 
                                (1.0f - integral_factor) * right_bands[i];
            
            if (smoothed_left < prev_left_spectrum[i]) {
                float fall = (prev_left_spectrum[i] - smoothed_left) * gravity_factor;
                prev_left_spectrum[i] -= fall;
                prev_left_spectrum[i] = std::max(prev_left_spectrum[i], smoothed_left);
            } else {
                prev_left_spectrum[i] = smoothed_left;
            }
            
            if (smoothed_right < prev_right_spectrum[i]) {
                float fall = (prev_right_spectrum[i] - smoothed_right) * gravity_factor;
                prev_right_spectrum[i] -= fall;
                prev_right_spectrum[i] = std::max(prev_right_spectrum[i], smoothed_right);
            } else {
                prev_right_spectrum[i] = smoothed_right;
            }
            
            left_out[i] = std::min(255, std::max(0, (int)prev_left_spectrum[i]));
            right_out[i] = std::min(255, std::max(0, (int)prev_right_spectrum[i]));
        }
        
        delete[] temp_left;
        delete[] temp_right;
    }
    
    void getVUMeterData(int& left_out, int& right_out) {
        std::array<int, 7> left_spectrum, right_spectrum;
        getSpectrumData(left_spectrum, right_spectrum);
        
        int left_sum = 0, right_sum = 0;
        for (int i = 0; i < 7; i++) {
            left_sum += left_spectrum[i];
            right_sum += right_spectrum[i];
        }
        
        left_out = left_sum / 7;
        right_out = right_sum / 7;
    }
    
    void getWaveformData(float* out, int samples, bool left_channel) {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        size_t read_pos = (write_pos + FFT_SIZE_BASS * 2 - samples) % (FFT_SIZE_BASS * 2);
        
        for (int i = 0; i < samples; i++) {
            out[i] = left_channel ? circular_buffer_left[read_pos] : circular_buffer_right[read_pos];
            read_pos = (read_pos + 1) % (FFT_SIZE_BASS * 2);
        }
    }
    
    void getStereoAnalysis(float& phase, float& correlation) {
        const int ANALYSIS_SAMPLES = 512;
        float left[ANALYSIS_SAMPLES], right[ANALYSIS_SAMPLES];
        
        {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            size_t read_pos = (write_pos + FFT_SIZE_BASS * 2 - ANALYSIS_SAMPLES) % (FFT_SIZE_BASS * 2);
            
            for (int i = 0; i < ANALYSIS_SAMPLES; i++) {
                left[i] = circular_buffer_left[read_pos];
                right[i] = circular_buffer_right[read_pos];
                read_pos = (read_pos + 1) % (FFT_SIZE_BASS * 2);
            }
        }
        
        // Calculate phase difference
        float sum_phase = 0.0f;
        for (int i = 0; i < ANALYSIS_SAMPLES; i++) {
            if (std::abs(left[i]) > 0.01f && std::abs(right[i]) > 0.01f) {
                sum_phase += atan2f(right[i], left[i]);
            }
        }
        phase = sum_phase / ANALYSIS_SAMPLES;
        
        // Calculate correlation
        float sum_l = 0, sum_r = 0, sum_lr = 0, sum_l2 = 0, sum_r2 = 0;
        for (int i = 0; i < ANALYSIS_SAMPLES; i++) {
            sum_l += left[i];
            sum_r += right[i];
            sum_lr += left[i] * right[i];
            sum_l2 += left[i] * left[i];
            sum_r2 += right[i] * right[i];
        }
        
        float n = ANALYSIS_SAMPLES;
        float num = n * sum_lr - sum_l * sum_r;
        float den = sqrtf((n * sum_l2 - sum_l * sum_l) * (n * sum_r2 - sum_r * sum_r));
        
        correlation = (den > 0) ? num / den : 0.0f;
        correlation = std::max(-1.0f, std::min(1.0f, correlation));
    }
    
    void setSensitivity(int value) {
        sensitivity = std::max(10.0f, std::min(300.0f, (float)value));
        updateParameters();
    }

    void setNoiseReduction(int value) {
        noise_reduction = std::max(0.0f, std::min(100.0f, (float)value));
        updateParameters();
    }
    
    int getSensitivity() const { return (int)sensitivity; }
    int getNoiseReduction() const { return (int)noise_reduction; }
};

// --- Improved TextScroller (Ping-Pong) ---
class TextScroller {
    private:
        std::string current_text;
        float scroll_position;
        std::chrono::steady_clock::time_point last_update_time;
        float wait_timer;
        int text_width_pixels;
        
        static constexpr float SCROLL_SPEED = 25.0f; // Pixels per second
        static constexpr float WAIT_TIME_SEC = 2.0f; // Pause at ends

        enum State { WAIT_START, SCROLL_RIGHT, WAIT_END, SCROLL_LEFT } state;

    public:
        TextScroller() : scroll_position(0), wait_timer(0), text_width_pixels(0), 
                        state(WAIT_START) {
            last_update_time = std::chrono::steady_clock::now();
        }
        
        void setText(const std::string& text) {
            if (text != current_text) {
                current_text = text;
                reset();
            }
        }

        void reset() {
            scroll_position = 0;
            wait_timer = 0;
            state = WAIT_START;
            text_width_pixels = 0; // Force recalculation
            last_update_time = std::chrono::steady_clock::now();
        }
        
        void render(Display* display, int x_offset, int y_offset, int max_width, FontManager* fm) {
            if (current_text.empty() || !fm) return;
            
            // Calculate timing
            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now - last_update_time).count();
            last_update_time = now;

            // Calculate width if needed
            if(text_width_pixels == 0) {
                text_width_pixels = fm->getTextWidth(current_text.c_str(), FontManager::SMALL);
            }
            
            int render_x = x_offset;

            // Logic: Only scroll if text is larger than the viewport
            if (text_width_pixels <= max_width) {
                // Center the text if it fits
                render_x = x_offset + (max_width - text_width_pixels) / 2;
                state = WAIT_START;
                scroll_position = 0;
            } else {
                // Max scrollable distance
                float max_scroll = (float)(text_width_pixels - max_width);

                switch (state) {
                    case WAIT_START:
                        scroll_position = 0;
                        wait_timer += dt;
                        if (wait_timer >= WAIT_TIME_SEC) {
                            wait_timer = 0;
                            state = SCROLL_RIGHT;
                        }
                        break;

                    case SCROLL_RIGHT:
                        scroll_position += SCROLL_SPEED * dt;
                        if (scroll_position >= max_scroll) {
                            scroll_position = max_scroll;
                            state = WAIT_END;
                        }
                        break;

                    case WAIT_END:
                        scroll_position = max_scroll;
                        wait_timer += dt;
                        if (wait_timer >= WAIT_TIME_SEC) {
                            wait_timer = 0;
                            state = SCROLL_LEFT;
                        }
                        break;

                    case SCROLL_LEFT:
                        scroll_position -= SCROLL_SPEED * dt;
                        if (scroll_position <= 0) {
                            scroll_position = 0;
                            state = WAIT_START;
                        }
                        break;
                }
                
                // Position for rendering (move text left by scroll_position)
                render_x = x_offset - (int)scroll_position;
            }

            // Render
            // FontManager clips internally, so negative X or X > 128 is safe
            fm->renderText(current_text.c_str(), display->buffer, Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT, 
                        render_x, y_offset, FontManager::SMALL);
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
        TextScroller title_scroller_left;
        TextScroller title_scroller_right;
        
        void drawTitle(Display* display, int y, bool is_left) {
            if (!font_manager) return;
            
            std::string txt;
            
            // Check if Bluetooth is active and has metadata
            if (bluetooth_monitor && bluetooth_monitor->isConnected()) {
                txt = bluetooth_monitor->getFormattedText();
            } else if (mpd_client) {
                txt = mpd_client->getFormattedText();
            }
            
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

// --- Concrete Visualizations ---
class VUMeterViz : public Visualization {
    public:
        struct DBPosition {
            int x;
            const char* text;
        };
        
        std::array<DBPosition, 11> db_positions;
        static constexpr const char* POWER_SCALE[6] = {"0", "20", "40", "60", "80", "100"};

        void calculateDBPositions() {
            // dB values matching Python code
            const float db_values[11] = {-20, -10, -7, -5, -3, -2, -1, 0, 1, 2, 3};
            
            for (int i = 0; i < 11; i++) {
                // Convert dB to linear value
                float value = powf(10.0f, db_values[i] / 20.0f);
                float log_pos = log10f(value);
                
                // Min/max log values for -20dB to +3dB
                float min_log = log10f(powf(10.0f, -20.0f / 20.0f));
                float max_log = log10f(powf(10.0f, 3.0f / 20.0f));
                
                // Calculate x position (0-125 range as in Python)
                int x_pos = (int)((log_pos - min_log) / (max_log - min_log) * 125.0f);
                
                // Store position and text
                db_positions[i].x = x_pos;
                // Static storage for text
                static char texts[11][4];
                snprintf(texts[i], 4, "%d", abs((int)db_values[i]));
                db_positions[i].text = texts[i];
            }
        }

        VUMeterViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l, r, m, bt, f) {
                calculateDBPositions();  // Initialize before any render() calls
        }
        const char* getName() const override { return "VU Meter"; }

        void render(ControlState&, AudioProcessor& audio) override {
            int l_vu, r_vu; audio.getVUMeterData(l_vu, r_vu);
            drawMeter(left_display, l_vu, "LEFT");
            drawMeter(right_display, r_vu, "RIGHT");
        }
        
        void drawMeter(Display* d, int val, const char* label) {
            d->clear();
            // Draw dB markings
            for (const auto& pos : db_positions) {
                d->drawText(pos.x, 5, pos.text, 0xFFFF, FontManager::SMALL);
                d->drawLine(pos.x, 7, pos.x, 9);
            }

            // Draw horizontal lines
            d->drawLine(108, 8, 127, 8);  // Short line at right
            d->drawLine(0, 9, 127, 9);    // Full line
            d->drawLine(0, 11, 127, 11);    // Full line
            
            // Draw edge marks
            d->drawLine(0, 6, 0, 8);
            d->drawLine(0, 11, 0, 13);
            d->drawLine(127, 6, 127, 8);
            d->drawLine(127, 11, 127, 13);
            
            // Draw power scale
            for (int i = 0; i < 6; i++) {
                int x = i * 22;
                d->drawText(x, 22, POWER_SCALE[i], 0xFFFF, FontManager::SMALL);
                d->drawLine(x, 11, x, 13);
            }
            
            // Draw +/- indicators
            d->drawText(0, 28, "-", 0xFFFF, FontManager::SMALL);
            d->drawText(124, 28, "+", 0xFFFF, FontManager::SMALL);
            
            // Draw channel label
            d->drawText(0, 128, label, 0xFFFF, FontManager::SMALL);
            d->drawText(120, 128, "dB", 0xFFFF, FontManager::SMALL);
            
            // Needle logic
            int pos = (val * 127) / 255;
            int start_x = 71 - (127 - pos) / 8;
            int curve = pos * (127 - pos);
            int end_y = 20 - curve / 200;
            d->drawLine(start_x, 128, pos, end_y, 0xF800);
            //d->display();
        }
};

class SpectrumViz : public Visualization {
    std::array<float, 7> peak_l{}, peak_r{};
    const char* LABELS[7] = {"63", "160", "400", "1K", "2.5K", "6.3K", "16K"};
    public:
        SpectrumViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l, r, m, bt, f) {}
        const char* getName() const override { return "Spectrum"; }
        
        void render(ControlState&, AudioProcessor& audio) override {
            std::array<int, 7> l, r; audio.getSpectrumData(l, r);
            drawSpec(left_display, l, peak_l, "SPEC L", true);
            drawSpec(right_display, r, peak_r, "SPEC R", false);
        }
        
        void drawSpec(Display* d, const std::array<int,7>& data, std::array<float,7>& peaks, const char* t, bool is_l) {
            d->clear();
            drawTitle(d, 5, is_l);
            for(int i=0; i<7; i++) {
                int x = 1 + i*19;
                int h = (data[i] * 110) / 255;
                int y = 120 - h;
                if(h>0) d->drawRect(x, y, 12, h, true);
                
                // Peak drop
                if(y < peaks[i]) peaks[i] = y;
                else peaks[i] = std::min(120.0f, peaks[i] + 0.5f);
                d->drawLine(x, (int)peaks[i], x+11, (int)peaks[i],0xF800);
                
                d->drawText(x, Config::SCREEN_HEIGHT, LABELS[i], 0xFFFF, FontManager::SMALL);
            }
        }
};

class EmptySpectrumViz : public Visualization {
    const char* LABELS[7] = {"63", "160", "400", "1K", "2.5K", "6.3K", "16K"};
    public:
        EmptySpectrumViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l, r, m, bt, f) {}
        const char* getName() const override { return "Empty Spectrum"; }
        
        void render(ControlState&, AudioProcessor& audio) override {
            std::array<int, 7> l, r; audio.getSpectrumData(l, r);
            drawSpec(left_display, l, "SPEC L", true);
            drawSpec(right_display, r, "SPEC R", false);
        }
        
        void drawSpec(Display* d, const std::array<int,7>& data, const char* t, bool is_l) {
            d->clear();
            drawTitle(d, 5, is_l);
            for(int i=0; i<7; i++) {
                int x = 1 + i*19;
                int h = (data[i] * 110) / 255;
                int y = 120 - h;
                int w = 12;
                
                if(h>0) {
                    d->drawLine(x, 120, x, y,0x07E0);         // Left
                    d->drawLine(x+w, 120, x+w, y, 0x07E0);     // Right
                    d->drawLine(x, y, x+w, y, 0x07E0);        // Top
                }
                d->drawText(x, Config::SCREEN_HEIGHT, LABELS[i],0xFFFF, FontManager::SMALL);
            }
        }
};

class TeubSpectrumViz : public Visualization {
    std::array<float, 7> peak_l{}, peak_r{};
    const char* LABELS[7] = {"63", "160", "400", "1K", "2.5K", "6.3K", "16K"};
    public:
        TeubSpectrumViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l, r, m, bt, f) {}
        const char* getName() const override { return "Teub Spectrum"; }
        
        void render(ControlState&, AudioProcessor& audio) override {
            std::array<int, 7> l, r; audio.getSpectrumData(l, r);
            drawSpec(left_display, l, peak_l, "TEUB L", true);
            drawSpec(right_display, r, peak_r, "TEUB R", false);
        }
        
        void drawSpec(Display* d, const std::array<int,7>& data, std::array<float,7>& peaks, const char* t, bool is_l) {
            d->clear();
            drawTitle(d, 5, is_l);
            int bar_bottom = 110;
            
            for(int i=0; i<7; i++) {
                int x = 1 + i*19;
                int h = (data[i] * (bar_bottom - 12)) / 255;
                int y = bar_bottom - h;
                y = std::max(y, 12);
                
                // Draw Phallic shape (Line + Circle tip)
                if(h>0) {
                    d->drawLine(x, y, x, bar_bottom);
                    d->drawLine(x+8, y, x+8, bar_bottom);
                    d->drawCircle(x+4, y, 5, false); // Tip
                    d->drawLine(x+4, y-3, x+4, y-1); // Detail
                }
                
                // Peak (Droplets)
                if(y < peaks[i]) peaks[i] = y;
                else peaks[i] = std::min((float)bar_bottom, peaks[i] + 0.8f);
                
                if(peaks[i] < bar_bottom) {
                    int py = (int)peaks[i];
                    d->drawLine(x+4, py-4, x+4, py-2);
                    d->drawLine(x+3, py-4, x+3, py-4);
                    d->drawLine(x+5, py-4, x+5, py-6);
                }
                
                // Balls
                d->drawCircle(x, 116, 5, false);
                d->drawCircle(x+8, 116, 5, false);
                
                d->drawText(x, Config::SCREEN_HEIGHT, LABELS[i], 0xFFFF, FontManager::SMALL);
            }
        }
};

class WaveformViz : public Visualization {
    float samples[128];
    public:
        WaveformViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) : Visualization(l, r, m, bt, f) {}
        const char* getName() const override { return "Waveform"; }
        
        void render(ControlState&, AudioProcessor& audio) override {
            drawWave(left_display, audio, true);
            drawWave(right_display, audio, false);
        }
        
        void drawWave(Display* d, AudioProcessor& a, bool is_left) {
            d->clear();
            drawTitle(d, 5, is_left);
            a.getWaveformData(samples, 128, is_left);
            int cy = 37;
            d->drawLine(0, cy, 127, cy);
            for(int i=0; i<127; i++) {
                int y1 = cy - (int)(samples[i]*25);
                int y2 = cy - (int)(samples[i+1]*25);
                d->drawLine(i, std::clamp(y1,12,63), i+1, std::clamp(y2,12,63));
            }
        }
};

struct Point3D {
    float x, y, z;
    
    Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    
    void rotateX(float angle) {
        float rad = angle;
        float newY = y * cosf(rad) - z * sinf(rad);
        float newZ = y * sinf(rad) + z * cosf(rad);
        y = newY;
        z = newZ;
    }
    
    void rotateY(float angle) {
        float rad = angle;
        float newX = x * cosf(rad) + z * sinf(rad);
        float newZ = -x * sinf(rad) + z * cosf(rad);
        x = newX;
        z = newZ;
    }
    
    void rotateZ(float angle) {
        float rad = angle;
        float newX = x * cosf(rad) - y * sinf(rad);
        float newY = x * sinf(rad) + y * cosf(rad);
        x = newX;
        y = newY;
    }
};

struct Point2D {
    int x, y;
    Point2D(int x = 0, int y = 0) : x(x), y(y) {}
};

class Grid3DViz : public Visualization {
    private:
        static constexpr int GRID_SIZE = 16;
        float time = 0.0f;
        
        Point2D projectGrid(float x, float y, float z, int cx, int cy) {
            float fov = 250.0f;
            float perspective = fov / (fov + z + 80.0f);
            return Point2D(
                cx + (int)(x * perspective),
                cy + (int)(y * perspective)
            );
        }
        
    public:
    Grid3DViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) 
        : Visualization(l, r, m, bt, f) {}
    
    const char* getName() const override { return "3D Grid"; }
    
    void render(ControlState& state, AudioProcessor& audio) override {
        std::array<int, 7> left_spectrum, right_spectrum;
        audio.getSpectrumData(left_spectrum, right_spectrum);
        
        float bass = ((left_spectrum[0] + left_spectrum[1]) / 2.0f) / 255.0f;
        float mid = ((left_spectrum[2] + left_spectrum[3]) / 2.0f) / 255.0f;
        
        time += 0.1f + bass * 0.2f;
        
        left_display->clear();
        
        // Dessiner la grille ondulante
        for (int gz = 0; gz < GRID_SIZE; gz++) {
            for (int gx = 0; gx < GRID_SIZE; gx++) {
                float x = (gx - GRID_SIZE / 2) * 8.0f;
                float z = gz * 8.0f - 50.0f;
                
                // Hauteur ondulante basée sur la position et le temps
                float wave = sinf(gx * 0.3f + time) * cosf(gz * 0.3f + time * 0.7f);
                float y = wave * 15.0f * (bass + 0.3f);
                
                Point2D p = projectGrid(x, y, z, 64, 48);
                
                // Dessiner ligne horizontale
                if (gx < GRID_SIZE - 1) {
                    float x2 = (gx + 1 - GRID_SIZE / 2) * 8.0f;
                    float wave2 = sinf((gx + 1) * 0.3f + time) * cosf(gz * 0.3f + time * 0.7f);
                    float y2 = wave2 * 15.0f * (bass + 0.3f);
                    Point2D p2 = projectGrid(x2, y2, z, 64, 48);
                    left_display->drawLine(p.x, p.y, p2.x, p2.y);
                }
                
                // Dessiner ligne verticale
                if (gz < GRID_SIZE - 1) {
                    float z2 = (gz + 1) * 8.0f - 50.0f;
                    float wave2 = sinf(gx * 0.3f + time) * cosf((gz + 1) * 0.3f + time * 0.7f);
                    float y2 = wave2 * 15.0f * (bass + 0.3f);
                    Point2D p2 = projectGrid(x, y2, z2, 64, 48);
                    left_display->drawLine(p.x, p.y, p2.x, p2.y);
                }
            }
        }
        
        drawTitle(left_display, 5, true);
        
        // ===== ÉCRAN DROITE: Vue de côté =====
        right_display->clear();
        
        // Profil de la vague
        for (int i = 0; i < 127; i++) {
            float t1 = i / 127.0f * 6.28f;
            float t2 = (i + 1) / 127.0f * 6.28f;
            float y1 = 32 + sinf(t1 * 2.0f + time) * 20.0f * (mid + 0.3f);
            float y2 = 32 + sinf(t2 * 2.0f + time) * 20.0f * (mid + 0.3f);
            right_display->drawLine(i, (int)y1, i + 1, (int)y2);
        }
        
        drawTitle(right_display, 5, false);
    }
};

class Particles3DViz : public Visualization {
    private:
        static constexpr int NUM_PARTICLES = 60;
        
        struct Particle {
            float x, y, z;
            float vx, vy, vz;
            int life;
        };
        
        std::array<Particle, NUM_PARTICLES> particles;
        float explosion_timer = 0.0f;
        
        void resetParticle(Particle& p) {
            // Position centrale
            p.x = (rand() % 20) - 10.0f;
            p.y = (rand() % 20) - 10.0f;
            p.z = (rand() % 20) - 10.0f;
            
            // Vitesse aléatoire
            p.vx = ((rand() % 200) - 100) / 50.0f;
            p.vy = ((rand() % 200) - 100) / 50.0f;
            p.vz = ((rand() % 200) - 100) / 50.0f;
            p.life = 100;
        }
        
    public:
    Particles3DViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) 
        : Visualization(l, r, m, bt, f) {
        for (auto& p : particles) {
            resetParticle(p);
        }
    }
    
    const char* getName() const override { return "Particles 3D"; }
    
    Point2D projectParticle(float x, float y, float z, int cx, int cy) {
        float fov = 200.0f;
        float perspective = fov / (fov + z + 100.0f);
        return Point2D(
            cx + (int)(x * perspective),
            cy + (int)(y * perspective)
        );
    }
    
    void render(ControlState& state, AudioProcessor& audio) override {
        std::array<int, 7> left_spectrum, right_spectrum;
        audio.getSpectrumData(left_spectrum, right_spectrum);
        
        float bass = ((left_spectrum[0] + left_spectrum[1]) / 2.0f) / 255.0f;
        float kick_threshold = 0.7f;
        
        // Détecter un "kick" (grosse basse)
        if (bass > kick_threshold) {
            explosion_timer = 20.0f;
        }
        
        // Mettre à jour les particules
        for (auto& p : particles) {
            if (explosion_timer > 0) {
                // Explosion !
                p.vx *= 1.3f;
                p.vy *= 1.3f;
                p.vz *= 1.3f;
            }
            
            // Mouvement
            p.x += p.vx;
            p.y += p.vy;
            p.z += p.vz;
            
            // Attraction vers le centre (douce)
            p.vx -= p.x * 0.01f;
            p.vy -= p.y * 0.01f;
            p.vz -= p.z * 0.01f;
            
            // Friction
            p.vx *= 0.98f;
            p.vy *= 0.98f;
            p.vz *= 0.98f;
            
            p.life--;
            if (p.life <= 0) {
                resetParticle(p);
            }
        }
        
        if (explosion_timer > 0) explosion_timer--;
        
        // ===== ÉCRAN GAUCHE: Vue 3D =====
        left_display->clear();
        
        for (const auto& p : particles) {
            Point2D proj = projectParticle(p.x, p.y, p.z, 64, 32);
            
            if (proj.x >= 0 && proj.x < Config::SCREEN_WIDTH && proj.y >= 0 && proj.y < Config::SCREEN_HEIGHT) {
                left_display->drawPixel(proj.x, proj.y);
                
                // Particules proches = plus grosses
                if (p.z > -20) {
                    left_display->drawPixel(proj.x + 1, proj.y);
                    left_display->drawPixel(proj.x, proj.y + 1);
                }
            }
        }
        
        drawTitle(left_display, 5, true);
        
        // ===== ÉCRAN DROITE: Vue de dessus (XZ) =====
        right_display->clear();
        
        for (const auto& p : particles) {
            int x = 64 + (int)(p.x * 1.5f);
            int y = 32 + (int)(p.z * 1.5f);
            
            if (x >= 0 && x < Config::SCREEN_WIDTH && y >= 0 && y < Config::SCREEN_HEIGHT) {
                right_display->drawPixel(x, y);
            }
        }
        
        // Cercle central
        left_display->drawCircle(64, 32, 5, false);
        
        drawTitle(right_display, 5, false);
    }
};

class TrackInfoViz : public Visualization {
    private:
        std::array<float, 7> peak_l{}, peak_r{};
        TextScroller info_scroller;
        
    public:
        TrackInfoViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f) 
            : Visualization(l, r, m, bt, f) {}
        
        const char* getName() const override { return "Track Info"; }
        
        void render(ControlState& state, AudioProcessor& audio) override {
            // ===== ÉCRAN GAUCHE: Informations de piste =====
            left_display->clear();
            
            int y = 5;
            
            // Check if Bluetooth is active
            bool bt_active = bluetooth_monitor && bluetooth_monitor->isConnected();
            
            if (bt_active) {
                // Bluetooth info
                bluetooth_monitor->updateElapsedTime();

                std::string info_line = bluetooth_monitor->getFormattedText();
                info_scroller.setText(info_line);
                info_scroller.render(left_display, 0, y, Config::SCREEN_WIDTH, font_manager);
                y += 10;
                
                // Source indicator
                left_display->drawText(0, y, "Source: Bluetooth", 0xFFFF, FontManager::SMALL);
                y += 10;
                
                // Codec info (if available in future)
                left_display->drawText(0, y, "A2DP SBC", 0xFFFF, FontManager::SMALL);
                y += 20;
                
                // Time (if available)
                uint32_t elapsed = bluetooth_monitor->getPosition() / 1000;
                uint32_t total = bluetooth_monitor->getDuration() / 1000;
                
                if (total > 0) {
                    char time_str[32];
                    snprintf(time_str, sizeof(time_str), "%02u:%02u / %02u:%02u", 
                            elapsed / 60, elapsed % 60, total / 60, total % 60);
                    left_display->drawText(0, y, time_str, 0xFFFF, FontManager::SMALL);
                    y += 5;
                    
                    // Progress bar
                    int bar_width = 126;
                    int progress_width = (elapsed * bar_width) / total;
                    left_display->drawRect(1, y, bar_width, 5, false);
                    if (progress_width > 2) {
                        left_display->drawRect(2, y+1, progress_width-2, 3, true);
                    }
                }
            } else if (mpd_client) {
                // MPD info (existing code)
                mpd_client->updateElapsedTime();
                
                std::stringstream info_line;
                info_line << mpd_client->getTrackNumber() << ". " 
                        << mpd_client->getTitle();
                
                std::string artist = mpd_client->getArtist();
                if (!artist.empty() && artist != "Unknown Artist") {
                    info_line << " - " << artist;
                }
                
                std::string album = mpd_client->getAlbum();
                if (!album.empty() && album != "Unknown Album") {
                    info_line << " - " << album;
                }
                
                std::string date = mpd_client->getDate();
                if (!date.empty()) {
                    info_line << " (" << date << ")";
                }
                
                info_scroller.setText(info_line.str());
                info_scroller.render(left_display, 0, y, Config::SCREEN_WIDTH, font_manager);
                y += 10;
                
                std::string bitrate = "Bitrate: " + mpd_client->getBitrate();
                left_display->drawText(0, y, bitrate.c_str(), 0xFFFF, FontManager::SMALL);
                y += 10;
                
                std::string format = mpd_client->getFormat();
                left_display->drawText(0, y, format.c_str(), 0xFFFF, FontManager::SMALL);
                y += 20;
                
                unsigned int elapsed = mpd_client->getElapsedTime();
                unsigned int total = mpd_client->getTotalTime();
                
                std::string time_str = mpd_client->formatTime(elapsed) + " / " + mpd_client->formatTime(total);
                left_display->drawText(0, y, time_str.c_str(), 0xFFFF, FontManager::SMALL);
                y += 5;
                
                if (total > 0) {
                    int bar_width = 126;
                    int progress_width = (elapsed * bar_width) / total;
                    left_display->drawRect(1, y, bar_width, 5, false);
                    if (progress_width > 2) {
                        left_display->drawRect(2, y+1, progress_width-2, 3, true);
                    }
                }
            }
            
            
            // ===== ÉCRAN DROITE: Spectre 14 bandes (unchanged) =====
            right_display->clear();
            
            std::array<int, 7> left_spectrum, right_spectrum;
            audio.getSpectrumData(left_spectrum, right_spectrum);
            
            int bar_width = 8;
            int spacing = 1;
            int bar_height = Config::SCREEN_HEIGHT;
            int bar_bottom = Config::SCREEN_HEIGHT;
            
            for (int i = 0; i < 7; i++) {
                int x_left = i * (bar_width + spacing);
                int h_left = (left_spectrum[i] * bar_height) / 255;
                int y_left = bar_bottom - h_left;
                
                if (h_left > 0) {
                    right_display->drawRect(x_left, y_left, bar_width, h_left, true);
                }
                
                if (y_left < peak_l[i]) peak_l[i] = y_left;
                else peak_l[i] = std::min((float)bar_bottom, peak_l[i] + 0.5f);
                right_display->drawLine(x_left, (int)peak_l[i], x_left + bar_width - 1, (int)peak_l[i]);
                
                int x_right = Config::SCREEN_HEIGHT + i * (bar_width + spacing);
                int h_right = (right_spectrum[i] * bar_height) / 255;
                int y_right = bar_bottom - h_right;
                
                if (h_right > 0) {
                    right_display->drawRect(x_right, y_right, bar_width, h_right, true);
                }
                
                if (y_right < peak_r[i]) peak_r[i] = y_right;
                else peak_r[i] = std::min((float)bar_bottom, peak_r[i] + 0.5f);
                right_display->drawLine(x_right, (int)peak_r[i], x_right + bar_width - 1, (int)peak_r[i]);
            }
            
        }
};

// Global flag for safe signal handling
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
      
    void onBluetoothConnectionChange(bool connected) {
        if (connected) {
            printf("Bluetooth connected - pausing MPD\n");
            system("mpc pause");
        } else {
            printf("Bluetooth disconnected\n");
        }
    }

    public:
    VisualizerApp() {
        // Initialize lgpio (replaces bcm2835_init + bcm2835_spi_begin)
        if (!LGpio::init()) {
            throw std::runtime_error("lgpio Init Failed");
        }
        
        d_left = std::make_unique<Display>(Config::LEFT_CS, Config::LEFT_DC, Config::LEFT_RST);
        d_right = std::make_unique<Display>(Config::RIGHT_CS, Config::RIGHT_DC, Config::RIGHT_RST);
        
        d_left->begin(); 
        d_right->begin();
        
        // Load fonts
        const char* fonts[] = {"/home/mlc/fonts/trixel-square.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", nullptr};
        for(int i=0; fonts[i]; i++) {
            if(font_mgr.init(fonts[i])) {
                d_left->setFont(&font_mgr); 
                d_right->setFont(&font_mgr);
                printf("Font loaded: %s\n", fonts[i]);
                break;
            }
        }
        
        mpd = std::make_unique<MPDClient>();
        mpd->start();
        
        bluetooth = std::make_unique<BluetoothMonitor>();
        bluetooth->start([this](bool connected) {
            onBluetoothConnectionChange(connected);
        });

        audio = std::make_unique<AudioProcessor>();
        
        vizs.push_back(std::make_unique<VUMeterViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<SpectrumViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<EmptySpectrumViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<TeubSpectrumViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<WaveformViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<Grid3DViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<Particles3DViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<TrackInfoViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        
        setupGPIO();
    }
    
    ~VisualizerApp() {
        printf("Shutting down...\n");
        
        audio->stop();
        bluetooth->stop();
        mpd->stop();
        
        d_left->shutdown();
        d_right->shutdown();
        
        LGpio::digitalWrite(Config::POWER_LED, 0);
        
        // Cleanup lgpio (replaces bcm2835_spi_end + bcm2835_close)
        LGpio::cleanup();
        
        printf("Displays powered off\n");
    }

    void setupGPIO() {
        // Setup input pins with pull-up
        // Replaces: bcm2835_gpio_fsel + bcm2835_gpio_set_pud
        uint8_t ins[] = {Config::VIZ_SW, Config::PLAY_SW, Config::POWER_SW, Config::FW_SW,Config::RW_SW};
        for(auto p : ins) { 
            LGpio::claimInputPullUp(p); 
        }
        
        // Setup power LED as output (replaces bcm2835_gpio_fsel + bcm2835_gpio_write)
        LGpio::claimOutput(Config::POWER_LED, 1);  // Start with LED on
        
    }
    
    void pollControls() {    
        // Calculate timing
        static auto last_btn_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_btn_time).count();

        // Replaces: bcm2835_gpio_lev (returns 0 for LOW, 1 for HIGH)
        if(LGpio::digitalRead(Config::VIZ_SW) == 0) {
            state.current_viz = (state.current_viz + 1) % vizs.size();
            printf("Switched to: %s\n", vizs[state.current_viz]->getName());
            LGpio::delay(200); // Debounce (replaces bcm2835_delay)
        }
        if(LGpio::digitalRead(Config::POWER_SW) == 0) state.running = false;

        // Playback controls with 200ms debounce
        if (elapsed > 200) {
            if (LGpio::digitalRead(Config::PLAY_SW) == 0) {
                printf("Play/Pause\n");
                mpd->playPause();
                last_btn_time = now;
            }
            
            if (LGpio::digitalRead(Config::FW_SW) == 0) {
                printf("Next track\n");
                mpd->nextTrack();
                last_btn_time = now;
            }
            
            if (LGpio::digitalRead(Config::RW_SW) == 0) {
                printf("Previous track\n");
                mpd->prevTrack();
                last_btn_time = now;
            }
        }

    }
    
    void run() {
        if(!audio->start()) {
            printf("Failed to start audio processor\n");
            return;
        }
        printf("Visualizer Running. Use Rotary 1 to Switch.\n");
        
        using namespace std::chrono;
        auto next_frame = steady_clock::now();
        int frame_count = 0;
        auto fps_timer = steady_clock::now();
        
        while(state.running && !g_shutdown_req) { 
            pollControls();
            
            bool has_audio = audio->checkForAudio();
            
            if(!has_audio && !state.is_sleeping) {
                state.is_sleeping = true;
                audio->setSleepState(true);
                d_left->sleep(); 
                d_right->sleep();
                LGpio::digitalWrite(Config::POWER_LED, 0);
                printf("Entering sleep mode\n");
            } else if (has_audio && state.is_sleeping) {
                state.is_sleeping = false;
                audio->setSleepState(false);
                d_left->wake(); 
                d_right->wake();
                LGpio::digitalWrite(Config::POWER_LED, 1);
                printf("Waking up\n");
            }
            
            if(!state.is_sleeping) {
                auto t1 = steady_clock::now();
                
                // Render (fills buffers, no display() calls inside)
                vizs[state.current_viz]->render(state, *audio);
                
                auto t2 = steady_clock::now();
                
                // SPI transfers
                d_left->display();
                d_right->display();
                
                auto t3 = steady_clock::now();
                
                frame_count++;
                if (duration_cast<seconds>(t3 - fps_timer).count() >= 1) {
                    long render_us = duration_cast<microseconds>(t2 - t1).count();
                    long spi_us = duration_cast<microseconds>(t3 - t2).count();
                    printf("FPS: %d, Render: %ldus, SPI: %ldus\n", frame_count, render_us, spi_us);
                    frame_count = 0;
                    fps_timer = t3;
                }
                
                next_frame += milliseconds(16);
                std::this_thread::sleep_until(next_frame);
            }
        }
    }
};

void sig_handler(int) { 
    g_shutdown_req = true; 
}

int main() {
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    
    try {
        VisualizerApp app;
        app.run();

    } catch(std::exception& e) {
        printf("Error: %s\n", e.what());
        LGpio::cleanup();  // Ensure cleanup on error
        return 1;
    }
    
    printf("Clean exit.\n");
    return 0;
}