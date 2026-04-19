/*
Dual SSD1309 OLED Audio Visualizer for Raspberry Pi 5
Optimized C++ implementation using lgpio library

sudo apt install g++ pkg-config libdbus-1-dev libasound2-dev libfftw3-dev libfreetype-dev libmpdclient-dev liblgpio-dev

g++ -o visualizer visualizer.cpp \
    -O3 -march=native -std=c++23 \
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
    //constexpr uint8_t POWER_SW = 13;
    constexpr uint8_t ROT_CLK = 17;
    constexpr uint8_t ROT_DT = 5;
    constexpr uint8_t ROT_SW = 13;
    constexpr int GPIO_CHIP = 4;        // RPi5 uses gpiochip4 for user GPIO
    constexpr int SPI_DEVICE = 0;       // /dev/spidev0.x
    constexpr int SPI_CHANNEL = 0;      // CS0 (we manage CS manually)
    constexpr int SPI_SPEED = 10000000;  // 10MHz
}

namespace LGpio {
    int gpio_handle = -1;
    int spi_handle_cs0 = -1;  // For LEFT display (CS0 = GPIO8)
    int spi_handle_cs1 = -1;  // For RIGHT display (CS1 = GPIO7)
    
    bool init() {
        // Open GPIO chip (RPi5 uses gpiochip4)
        gpio_handle = lgGpiochipOpen(Config::GPIO_CHIP);
        if (gpio_handle < 0) {
            // Try gpiochip0 as fallback (older kernels or RPi4)
            gpio_handle = lgGpiochipOpen(0);
            if (gpio_handle < 0) {
                printf("Failed to open GPIO chip: %s\n", lguErrorText(gpio_handle));
                return false;
            }
            printf("Using gpiochip0 (fallback)\n");
        } else {
            printf("Using gpiochip4 (RPi5)\n");
        }
        
        // Open SPI device for CS0 (LEFT display - GPIO8)
        spi_handle_cs0 = lgSpiOpen(Config::SPI_DEVICE, 0, Config::SPI_SPEED, 0);
        if (spi_handle_cs0 < 0) {
            printf("Failed to open SPI CS0: %s\n", lguErrorText(spi_handle_cs0));
            lgGpiochipClose(gpio_handle);
            gpio_handle = -1;
            return false;
        }
        
        // Open SPI device for CS1 (RIGHT display - GPIO7)
        spi_handle_cs1 = lgSpiOpen(Config::SPI_DEVICE, 1, Config::SPI_SPEED, 0);
        if (spi_handle_cs1 < 0) {
            printf("Failed to open SPI CS1: %s\n", lguErrorText(spi_handle_cs1));
            lgSpiClose(spi_handle_cs0);
            lgGpiochipClose(gpio_handle);
            gpio_handle = -1;
            spi_handle_cs0 = -1;
            return false;
        }
        
        printf("SPI initialized: CS0 and CS1 at %d Hz (hardware CS mode)\n", Config::SPI_SPEED);
        return true;
    }
    
    void cleanup() {
        if (spi_handle_cs0 >= 0) {
            lgSpiClose(spi_handle_cs0);
            spi_handle_cs0 = -1;
        }
        if (spi_handle_cs1 >= 0) {
            lgSpiClose(spi_handle_cs1);
            spi_handle_cs1 = -1;
        }
        if (gpio_handle >= 0) {
            lgGpiochipClose(gpio_handle);
            gpio_handle = -1;
        }
    }
    
    // GPIO helper functions
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
    
    // SPI transfer single byte to specific CS
    inline uint8_t spiTransfer(int cs_pin, uint8_t data) {
        char tx = data;
        char rx = 0;
        int handle = (cs_pin == Config::LEFT_CS) ? spi_handle_cs0 : spi_handle_cs1;
        lgSpiXfer(handle, &tx, &rx, 1);
        return rx;
    }
    
    // SPI write buffer to specific CS
    inline void spiWrite(int cs_pin, const char* buffer, int len) {
        int handle = (cs_pin == Config::LEFT_CS) ? spi_handle_cs0 : spi_handle_cs1;
        lgSpiWrite(handle, buffer, len);
    }
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
            withMPD([this](auto* c) {
                if (is_playing) mpd_run_toggle_pause(c);
                else mpd_run_play(c);
            });
        }

        void nextTrack() { withMPD([](auto* c) { mpd_run_next(c); }); }
        void prevTrack() { withMPD([](auto* c) { mpd_run_previous(c); }); }
        
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
        // --- AJOUT: Structure pour les pistes ---
        struct TrackInfo {
            std::string title;
            std::string artist;
            std::string file;
            std::string track_num;
        };

        // --- AJOUT: Récupère la liste triée des albums ---
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

        // --- AJOUT: Récupère les pistes d'un album ---
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
                    const char* title  = mpd_song_get_tag(song, MPD_TAG_TITLE,  0);
                    const char* artist = mpd_song_get_tag(song, MPD_TAG_ARTIST, 0);
                    const char* track  = mpd_song_get_tag(song, MPD_TAG_TRACK,  0);
                    ti.title  = title  ? title  : mpd_song_get_uri(song);
                    ti.artist = artist ? artist : "";
                    ti.file   = mpd_song_get_uri(song);
                    ti.track_num = track ? track : "";
                    // Nettoie "N/Total" → "N"
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
                mpd_search_commit(c);
                mpd_response_finish(c);
            });
        }

        void clearAndPlayAlbum(const std::string& album) {
            withMPD([&](auto* c) {
                mpd_run_clear(c);
                mpd_search_add_db_songs(c, true);
                mpd_search_add_tag_constraint(c, MPD_OPERATOR_DEFAULT, MPD_TAG_ALBUM, album.c_str());
                mpd_search_commit(c);
                mpd_response_finish(c);
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
                if (status) {
                    vol = (int)mpd_status_get_volume(status);
                    mpd_status_free(status);
                }
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

// --- Font Manager ---
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
            int baseline_y = y;
            
            const unsigned char* utext = (const unsigned char*)text;
            
            while (*utext) {
                // Décoder le caractère UTF-8
                unsigned int codepoint = 0;
                int bytes = 0;
                
                if ((*utext & 0x80) == 0x00) {
                    // 1 octet ASCII: 0xxxxxxx
                    codepoint = *utext;
                    bytes = 1;
                } else if ((*utext & 0xE0) == 0xC0) {
                    // 2 octets: 110xxxxx 10xxxxxx
                    codepoint = (*utext & 0x1F) << 6;
                    codepoint |= (*(utext+1) & 0x3F);
                    bytes = 2;
                } else if ((*utext & 0xF0) == 0xE0) {
                    // 3 octets: 1110xxxx 10xxxxxx 10xxxxxx
                    codepoint = (*utext & 0x0F) << 12;
                    codepoint |= (*(utext+1) & 0x3F) << 6;
                    codepoint |= (*(utext+2) & 0x3F);
                    bytes = 3;
                } else if ((*utext & 0xF8) == 0xF0) {
                    // 4 octets: 11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
                    codepoint = (*utext & 0x07) << 18;
                    codepoint |= (*(utext+1) & 0x3F) << 12;
                    codepoint |= (*(utext+2) & 0x3F) << 6;
                    codepoint |= (*(utext+3) & 0x3F);
                    bytes = 4;
                } else {
                    // Octet invalide, sauter
                    utext++;
                    continue;
                }
                
                // Charger et rendre le glyphe avec le codepoint Unicode
                if (FT_Load_Char(face, codepoint, FT_LOAD_RENDER)) { 
                    utext += bytes; 
                    continue; 
                }
                
                FT_GlyphSlot slot = face->glyph;
                FT_Bitmap* bitmap = &slot->bitmap;
                
                int start_x = cursor_x + slot->bitmap_left;
                int start_y = baseline_y - slot->bitmap_top;
                
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
                utext += bytes;  // Avancer du nombre d'octets UTF-8
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
                // Décoder le caractère UTF-8
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

// --- Display Class ---
class Display {
    private:
        uint8_t _cs, _dc, _rst;
        FontManager* font_manager;
        uint32_t active_frames = 0;
        bool is_sleeping = false;

    public:
    uint8_t buffer[Config::SPI_CHUNK_SIZE];

    Display(uint8_t cs, uint8_t dc, uint8_t rst) : _cs(cs), _dc(dc), _rst(rst), font_manager(nullptr), is_sleeping(false) {
        memset(buffer, 0x00, sizeof(buffer));
    }
    
    bool begin() {
        // Setup DC and RST as outputs (CS is handled by hardware SPI)
        LGpio::claimOutput(_dc, 0);
        LGpio::claimOutput(_rst, 1);
        
        // Reset sequence
        LGpio::digitalWrite(_rst, 0);
        LGpio::delay(10);
        LGpio::digitalWrite(_rst, 1);
        LGpio::delay(10);
        
        // SSD1309 initialization commands
        const uint8_t init_cmds[] = {
            0xAE,       // Display OFF
            0x20, 0x00, // Horizontal addressing mode
            0xB0,       // Start page address
            0xC8,       // COM output scan direction (remapped)
            0x00,       // Low column address
            0x10,       // High column address
            0x40,       // Start line address
            0x81, 0xFF, // Contrast control
            0xA1,       // Segment re-map
            0xA6,       // Normal display
            0xA8, 0x3F, // Multiplex ratio (64)
            0xA4,       // Output follows RAM content
            0xD3, 0x00, // Display offset
            0xD5, 0x70, // Display clock divide ratio ← Datasheet default 0x70
            0xD9, 0x22, // Pre-charge period ← Datasheet default 0x22
            0xDA, 0x12, // COM pins configuration
            0xDB, 0x34, // VCOMH deselect level ← Datasheet default 0x34
            0x8D, 0x14, // Enable charge pump
            0xAF        // Display ON
        };
        
            for(uint8_t cmd : init_cmds) sendCommand(cmd);
            return true;
    }
    
    void sendCommand(uint8_t cmd) {
        LGpio::digitalWrite(_dc, 0);  // Command mode
        LGpio::spiTransfer(_cs, cmd); // Hardware CS is automatic
    }
    
    void display() {
        sendCommand(0xB0);  // Start at page 0
        sendCommand(0x00);  // Column low
        sendCommand(0x10);  // Column high
        
        LGpio::digitalWrite(_dc, 1);  // Data mode
        LGpio::spiWrite(_cs, (char*)buffer, sizeof(buffer)); // Hardware CS is automatic
    }
    
    void clear() { memset(buffer, 0x00, sizeof(buffer)); }

    void sleep() {
        sendCommand(0xAE);// Display OFF
        is_sleeping = true;
    } 
    
    void shutdown() {
        sendCommand(0xAE);  // Display OFF
        clear();
        display();
    }

    void wake() { 
        sendCommand(0xAF);
        is_sleeping = false;
    }

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
            int x_start = std::max(0, x);
            int y_start = std::max(0, y);
            int x_end   = std::min(Config::SCREEN_WIDTH, x + w);
            int y_end   = std::min(Config::SCREEN_HEIGHT, y + h);

            for (int py = y_start; py < y_end; py++) {
                int row_offset = (py / 8) * Config::SCREEN_WIDTH;
                uint8_t bit_mask = (1 << (py % 8));
                for (int px = x_start; px < x_end; px++) {
                    buffer[row_offset + px] |= bit_mask;
                }
            }
        } else {
            drawLine(x, y, x + w - 1, y);
            drawLine(x + w - 1, y, x + w - 1, y + h - 1);
            drawLine(x + w - 1, y + h - 1, x, y + h - 1);
            drawLine(x, y + h - 1, x, y);
        }
    }

    void drawCircle(int x0, int y0, int radius, bool filled = false) {
        int x = radius, y = 0, err = 0;
        while (x >= y) {
            if (filled) {
                drawLine(x0 - x, y0 + y, x0 + x, y0 + y);
                drawLine(x0 - x, y0 - y, x0 + x, y0 - y);
                drawLine(x0 - y, y0 + x, x0 + y, y0 + x);
                drawLine(x0 - y, y0 - x, x0 + y, y0 - x);
            } else {
                drawPixel(x0 + x, y0 + y); drawPixel(x0 + y, y0 + x);
                drawPixel(x0 - y, y0 + x); drawPixel(x0 - x, y0 + y);
                drawPixel(x0 - x, y0 - y); drawPixel(x0 - y, y0 - x);
                drawPixel(x0 + y, y0 - x); drawPixel(x0 + x, y0 - y);
            }
            if (err <= 0) { y += 1; err += 2 * y + 1; }
            if (err > 0) { x -= 1; err -= 2 * x + 1; }
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
        
        struct FreqBand {
            int low, high;
            float correction;
        };
        
        static constexpr FreqBand FREQ_BANDS[7] = {
            {63, 120, 0.7f}, {120, 350, 1.2f}, {350, 900, 2.0f},
            {900, 2000, 6.0f}, {2000, 5000, 11.0f},
            {5000, 10000, 30.0f}, {10000, 16000, 50.0f}
        };

        // For each band here, a manual calibration was applied playing white and pink noise
        static constexpr FreqBand FREQ_BANDS_LARGE[64] = {
            {63, 71, 0.7f},
            {71, 80, 0.75f},
            {80, 90, 0.8f},
            {90, 100, 0.85f},
            {100, 112, 0.9f},
            {112, 125, 0.95f},
            {125, 140, 1.0f},
            {140, 160, 1.05f},
            {160, 180, 1.1f},
            {180, 200, 1.15f},
            {200, 224, 1.2f},
            {224, 250, 1.25f},
            {250, 280, 1.3f},
            {280, 315, 1.35f},
            {315, 355, 1.4f},
            {355, 400, 1.45f},
            {400, 450, 1.5f},
            {450, 500, 1.6f},
            {500, 560, 1.7f},
            {560, 630, 1.8f},
            {630, 710, 1.9f},
            {710, 800, 2.0f},
            {800, 900, 2.2f},
            {900, 1000, 2.4f},
            {1000, 1120, 2.6f},
            {1120, 1250, 2.8f},
            {1250, 1400, 3.0f},
            {1400, 1600, 3.3f},
            {1600, 1800, 3.6f},
            {1800, 2000, 4.0f},
            {2000, 2240, 4.4f},
            {2240, 2500, 4.8f},
            {2500, 2800, 5.3f},
            {2800, 3150, 5.8f},
            {3150, 3550, 6.4f},
            {3550, 4000, 7.0f},
            {4000, 4500, 7.7f},
            {4500, 5000, 8.5f},
            {5000, 5600, 9.3f},
            {5600, 6300, 10.2f},
            {6300, 7100, 11.2f},
            {7100, 8000, 12.3f},
            {8000, 9000, 13.5f},
            {9000, 10000, 14.8f},
            {10000, 10600, 16.2f},
            {10600, 11200, 17.7f},
            {11200, 11800, 19.4f},
            {11800, 12500, 21.2f},
            {12500, 13200, 23.2f},
            {13200, 13900, 25.4f},
            {13900, 14600, 27.8f},
            {14600, 15000, 30.4f},
            {15000, 15200, 33.3f},
            {15200, 15400, 36.4f},
            {15400, 15550, 39.8f},
            {15550, 15700, 41.5f},
            {15700, 15800, 43.3f},
            {15800, 15875, 45.1f},
            {15875, 15925, 47.0f},
            {15925, 15962, 48.5f},
            {15962, 15981, 49.3f},
            {15981, 15990, 49.6f},
            {15990, 15995, 49.8f},
            {15995, 16000, 50.0f}
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

        std::array<float, 64> prev_left_spectrum_large{};
        std::array<float, 64> prev_right_spectrum_large{};

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

        void fillTempBuffers() {
            std::lock_guard<std::mutex> lock(buffer_mutex);
            size_t read_pos = (write_pos + FFT_SIZE_BASS * 2 - FFT_SIZE_BASS) % (FFT_SIZE_BASS * 2);
            for (int i = 0; i < FFT_SIZE_BASS; i++) {
                temp_left[i] = circular_buffer_left[read_pos];
                temp_right[i] = circular_buffer_right[read_pos];
                read_pos = (read_pos + 1) % (FFT_SIZE_BASS * 2);
            }
        }

        template<typename BandType>
        void computeChannel(const float* samples, float* bands,
                            const BandType* freq_table, int num_bands,
                            int bass_end, int mid_end) {
            // BASS FFT
            for (int i = 0; i < FFT_SIZE_BASS; i++)
                fft_in_bass[i] = samples[i] * window_bass[i];
            fftwf_execute(plan_bass);

            for (int i = 0; i < bass_end; i++) {
                int low_idx = freq_table[i].low * FFT_SIZE_BASS / SAMPLE_RATE;
                int high_idx = freq_table[i].high * FFT_SIZE_BASS / SAMPLE_RATE;
                float sum = 0.0f;
                for (int j = low_idx; j < high_idx && j < FFT_SIZE_BASS/2; j++)
                    sum += fft_out_bass[j][0] * fft_out_bass[j][0] +
                        fft_out_bass[j][1] * fft_out_bass[j][1];
                bands[i] = sqrtf(sum / (high_idx - low_idx)) * scale_factor * freq_table[i].correction;
            }

            // MID FFT
            for (int i = 0; i < FFT_SIZE_MID; i++)
                fft_in_mid[i] = samples[i] * window_mid[i];
            fftwf_execute(plan_mid);

            for (int i = bass_end; i < mid_end; i++) {
                int low_idx = freq_table[i].low * FFT_SIZE_MID / SAMPLE_RATE;
                int high_idx = freq_table[i].high * FFT_SIZE_MID / SAMPLE_RATE;
                float sum = 0.0f;
                for (int j = low_idx; j < high_idx && j < FFT_SIZE_MID/2; j++)
                    sum += fft_out_mid[j][0] * fft_out_mid[j][0] +
                        fft_out_mid[j][1] * fft_out_mid[j][1];
                bands[i] = sqrtf(sum / (high_idx - low_idx)) * scale_factor * freq_table[i].correction;
            }

            // TREBLE FFT
            for (int i = 0; i < FFT_SIZE_TREBLE; i++)
                fft_in_treble[i] = samples[i] * window_treble[i];
            fftwf_execute(plan_treble);

            for (int i = mid_end; i < num_bands; i++) {
                int low_idx = freq_table[i].low * FFT_SIZE_TREBLE / SAMPLE_RATE;
                int high_idx = freq_table[i].high * FFT_SIZE_TREBLE / SAMPLE_RATE;
                if (high_idx <= low_idx) high_idx = low_idx + 1;
                float sum = 0.0f;
                for (int j = low_idx; j < high_idx && j < FFT_SIZE_TREBLE/2; j++)
                    sum += fft_out_treble[j][0] * fft_out_treble[j][0] +
                        fft_out_treble[j][1] * fft_out_treble[j][1];
                bands[i] = sqrtf(sum / (high_idx - low_idx)) * scale_factor * freq_table[i].correction;
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
            int buffer_size = FFT_SIZE_BASS * 2;
            circular_buffer_left = new float[buffer_size]();
            circular_buffer_right = new float[buffer_size]();
            
            temp_left = new float[FFT_SIZE_BASS];
            temp_right = new float[FFT_SIZE_BASS];

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
            delete[] temp_left;
            delete[] temp_right;
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
            int err = snd_pcm_open(&pcm_handle, "rav", SND_PCM_STREAM_CAPTURE, 0);
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
            snd_pcm_hw_params_set_buffer_size(pcm_handle, hw_params, 8192);
            snd_pcm_hw_params_set_period_size(pcm_handle, hw_params, 2048, 0);
            
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

        void resetSleepTimer() {
            last_audio_time = std::chrono::steady_clock::now();
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

            fillTempBuffers();
            computeChannel(temp_left,  left_bands.data(),  FREQ_BANDS, 7, 3, 5);
            computeChannel(temp_right, right_bands.data(), FREQ_BANDS, 7, 3, 5);
            applySmoothingAndOutput<7>(prev_left_spectrum.data(),  left_bands.data(),  left_out.data());
            applySmoothingAndOutput<7>(prev_right_spectrum.data(), right_bands.data(), right_out.data());
        }

        void getLargeSpectrumData(std::array<int, 64>& left_out, std::array<int, 64>& right_out) {
            std::array<float, 64> left_bands{}, right_bands{};

            fillTempBuffers();
            computeChannel(temp_left,  left_bands.data(),  FREQ_BANDS_LARGE, 64, 24, 44);
            computeChannel(temp_right, right_bands.data(), FREQ_BANDS_LARGE, 64, 24, 44);
            applySmoothingAndOutput<64>(prev_left_spectrum_large.data(),  left_bands.data(),  left_out.data());
            applySmoothingAndOutput<64>(prev_right_spectrum_large.data(), right_bands.data(), right_out.data());
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

// --- TextScroller (Ping-Pong) ---
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

// --- Visualizations ---
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
                d->drawText(pos.x, 5, pos.text, FontManager::SMALL);
                d->drawLine(pos.x, 7, pos.x, 9);
            }

            // Draw horizontal lines
            d->drawLine(108, 8, 127, 8);  // Short line at right
            d->drawLine(0, 9, 127, 9);    // Full line
            d->drawLine(0, 11, 110, 11);    // Full line
            
            // Draw edge marks
            d->drawLine(0, 6, 0, 8);
            d->drawLine(0, 11, 0, 13);
            d->drawLine(127, 6, 127, 8);
            //d->drawLine(127, 11, 127, 13);
            
            // Draw power scale
            for (int i = 0; i < 6; i++) {
                int x = i * 22;
                d->drawText(x, 22, POWER_SCALE[i], FontManager::SMALL);
                d->drawLine(x, 11, x, 13);
            }
            
            // Draw +/- indicators
            d->drawText(0, 28, "-", FontManager::SMALL);
            d->drawText(124, 28, "+", FontManager::SMALL);
            
            // Draw channel label
            d->drawText(0, 64, label, FontManager::SMALL);
            d->drawText(120, 64, "dB", FontManager::SMALL);
            
            // Needle logic
            int pos = (val * 127) / 255;
            int start_x = 71 - (127 - pos) / 8;
            int curve = pos * (127 - pos);
            int end_y = 20 - curve / 200;
            d->drawLine(start_x, 63, pos, end_y);
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
                int h = (data[i] * 49) / 255;
                int y = 57 - h;
                if(h>0) d->drawRect(x, y, 12, h, true);
                
                // Peak drop
                if(y < peaks[i]) peaks[i] = y;
                else peaks[i] = std::min(56.0f, peaks[i] + 0.5f);
                d->drawLine(x, (int)peaks[i], x+11, (int)peaks[i]);
                
                d->drawText(x, 64, LABELS[i], FontManager::SMALL);
            }
            //d->display();
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
                int h = (data[i] * 49) / 255;
                int y = 57 - h;
                int w = 12;
                
                if(h>0) {
                    d->drawLine(x, 57, x, y);         // Left
                    d->drawLine(x+w, 57, x+w, y);     // Right
                    d->drawLine(x, y, x+w, y);        // Top
                }
                d->drawText(x, 64, LABELS[i], FontManager::SMALL);
            }
            //d->display();
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
            int bar_bottom = 47;
            
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
                d->drawCircle(x, 52, 5, false);
                d->drawCircle(x+8, 52, 5, false);
                
                d->drawText(x, 64, LABELS[i], FontManager::SMALL);
            }
            //d->display();
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
            //d->display();
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
                info_scroller.render(left_display, 1, y, Config::SCREEN_WIDTH, font_manager);
                y += 10;
                
                // Source indicator
                left_display->drawText(1, y, "Source: Bluetooth", FontManager::SMALL);
                y += 10;
                
                // Codec info (if available in future)
                left_display->drawText(1, y, "A2DP SBC", FontManager::SMALL);
                y += 20;
                
                // Time (if available)
                uint32_t elapsed = bluetooth_monitor->getPosition() / 1000;
                uint32_t total = bluetooth_monitor->getDuration() / 1000;
                
                if (total > 0) {
                    char time_str[32];
                    snprintf(time_str, sizeof(time_str), "%02u:%02u / %02u:%02u", 
                            elapsed / 60, elapsed % 60, total / 60, total % 60);
                    left_display->drawText(1, y, time_str, FontManager::SMALL);
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
                unsigned int elapsed = mpd_client->getElapsedTime();
                unsigned int total = mpd_client->getTotalTime();
                
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
                info_scroller.render(left_display, 1, y, (Config::SCREEN_WIDTH)-2, font_manager);
                y += 10;
                
                std::string bitrate = "Bitrate: " + mpd_client->getBitrate();
                left_display->drawText(1, y, bitrate.c_str(), FontManager::SMALL);
                y += 10;
                
                std::string format = mpd_client->getFormat();
                left_display->drawText(1, y, format.c_str(), FontManager::SMALL);
                y += 20;
                               
                std::string time_str = mpd_client->formatTime(elapsed) + " / " + mpd_client->formatTime(total);
                left_display->drawText(1, y, time_str.c_str(), FontManager::SMALL);
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
        
        std::array<int, 7> left_spectrum, right_spectrum;
        audio.getSpectrumData(left_spectrum, right_spectrum);
        
        int bar_width = 2;
        int spacing = 1;
        int bar_height = 32;
        int bar_bottom = 43;
        
        for (int i = 0; i < 7; i++) {
            int x_left = 80+(i * (bar_width + spacing));
            int h_left = (left_spectrum[i] * bar_height) / 255;
            int y_left = bar_bottom - h_left;
            
            if (h_left > 0) {
                left_display->drawRect(x_left, y_left, bar_width, h_left, true);
            }
            
            if (y_left < peak_l[i]) peak_l[i] = y_left;
            else peak_l[i] = std::min((float)bar_bottom, peak_l[i] + 0.5f);
            left_display->drawLine(x_left, (int)peak_l[i], x_left + bar_width - 1, (int)peak_l[i]);
            
            int x_right = 80+16+7 + (i * (bar_width + spacing));
            int h_right = (right_spectrum[i] * bar_height) / 255;
            int y_right = bar_bottom - h_right;
            
            if (h_right > 0) {
                left_display->drawRect(x_right, y_right, bar_width, h_right, true);
            }
            
            if (y_right < peak_r[i]) peak_r[i] = y_right;
            else peak_r[i] = std::min((float)bar_bottom, peak_r[i] + 0.5f);
            left_display->drawLine(x_right, (int)peak_r[i], x_right + bar_width - 1, (int)peak_r[i]);
        }

        left_display->drawLine(77, 13, 77, 45);
        left_display->drawLine(77, 45, 125, 45);

        // ===== ÉCRAN DROITE: Elapsed time =====
        right_display->clear();

        std::array<int, 64> left_spectrum2, right_spectrum2;
        audio.getLargeSpectrumData(left_spectrum2, right_spectrum2);

        int bar_width_2 = 1;
        int spacing_2 = 1;
        int bar_height_2 = 31;
        int bar_height_3 = 31;
        int bar_bottom_2 = 32;
        int bar_bottom_3 = 32;

        
        for (int i = 0; i < 64; i++) {
            int x_left_2 = (i * (bar_width_2 + spacing_2));
            int h_left_2 = 1+(left_spectrum2[i] * bar_height_2) / 255;
            int y_left_2 = bar_bottom_2 - h_left_2;
            
            if (h_left_2 > 0) {
                right_display->drawRect(x_left_2, y_left_2, bar_width_2, h_left_2, true);
            }
            
           
            int x_right_2 = (i * (bar_width_2 + spacing_2));
            int h_right_2 = 1+(right_spectrum2[i] * bar_height_3) / 255;
            int y_right_2 = bar_bottom_3;
            
            if (h_right_2 > 0) {
                right_display->drawRect(x_right_2, y_right_2, bar_width_2, h_right_2, true);
            }

        }

    }



};

// --- Playlist Editor Visualization ---
class PlaylistEditorViz : public Visualization {
    enum class EditorState { BROWSE_ALBUMS, BROWSE_TRACKS, FEEDBACK };

    EditorState  editor_state   = EditorState::BROWSE_ALBUMS;
    bool         albums_loaded  = false;
    bool         loading        = false;

    std::vector<std::string>         albums;
    std::vector<MPDClient::TrackInfo> tracks;

    int album_cursor = 0, album_scroll = 0;
    int track_cursor = 0, track_scroll = 0;

    std::string selected_album;
    std::string feedback_msg;
    std::chrono::steady_clock::time_point feedback_ts;

    std::thread   load_thread;
    std::mutex    data_mtx;

    // État précédent des boutons (pour détection front montant)
    bool prev_up = false, prev_dn = false, prev_sel = false, prev_back = false;

    static constexpr int VISIBLE_LINES = 6;
    static constexpr int LINE_H        = 9;
    static constexpr int LIST_Y0       = 18; // y de la 1ère ligne
    static constexpr int HEADER_Y      = 7;

    // -------- helpers --------

    std::string truncatePx(const std::string& s, int max_px) {
        if (!font_manager) return s.substr(0, 20);
        if (font_manager->getTextWidth(s.c_str(), FontManager::SMALL) <= max_px)
            return s;
        std::string r = s;
        while (r.size() > 2 &&
               font_manager->getTextWidth((r + "..").c_str(), FontManager::SMALL) > max_px)
            r.pop_back();
        return r + "..";
    }

    void drawSelectionRow(Display* d, int row_idx, const std::string& label, bool selected) {
        int y = LIST_Y0 + row_idx * LINE_H;
        if (selected) {
            d->drawRect(0, y - 7, 126, 8, true);
            if (font_manager)
                font_manager->renderText(truncatePx(label, 120).c_str(),
                    d->buffer, Config::SCREEN_WIDTH, Config::SCREEN_HEIGHT,
                    2, y, FontManager::SMALL, /*invert=*/true);
        } else {
            d->drawText(2, y, truncatePx(label, 120).c_str(), FontManager::SMALL);
        }
    }

    void drawScrollbar(Display* d, int total, int visible, int scroll_offset) {
        if (total <= visible) return;
        int bar_top = LIST_Y0 - 7;
        int bar_bot = LIST_Y0 + visible * LINE_H - 1;
        int bar_h   = bar_bot - bar_top;
        int thumb_h = std::max(4, bar_h * visible / total);
        int thumb_y = bar_top + bar_h * scroll_offset / total;
        d->drawLine(127, bar_top, 127, bar_bot);
        d->drawRect(126, thumb_y, 2, thumb_h, true);
    }

    void setFeedback(const std::string& msg) {
        feedback_msg = msg;
        feedback_ts  = std::chrono::steady_clock::now();
        editor_state = EditorState::FEEDBACK;
    }

    void startLoadAlbums() {
        if (loading) return;
        loading       = true;
        albums_loaded = false;
        if (load_thread.joinable()) load_thread.join();
        load_thread = std::thread([this]() {
            auto res = mpd_client->getAlbums();
            { std::lock_guard<std::mutex> lk(data_mtx); albums = std::move(res); }
            albums_loaded = true;
            loading       = false;
        });
    }

    void startLoadTracks(const std::string& album) {
        if (loading) return;
        loading = true;
        if (load_thread.joinable()) load_thread.join();
        load_thread = std::thread([this, album]() {
            auto res = mpd_client->getAlbumTracks(album);
            { std::lock_guard<std::mutex> lk(data_mtx); tracks = std::move(res); }
            loading = false;
        });
    }

    // -------- rendu écran gauche (browser) --------

    void renderBrowser(Display* d) {
        d->clear();

        if (!albums_loaded && loading) {
            d->drawText(25, 35, "Chargement...", FontManager::SMALL);
            return;
        }

        if (editor_state == EditorState::FEEDBACK) {
            int elapsed_ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - feedback_ts).count();
            if (elapsed_ms > 1500)
                editor_state = EditorState::BROWSE_ALBUMS;

            // Boite centrée
            d->drawRect(8, 22, 112, 20, false);
            d->drawRect(9, 23, 110, 18, false);
            int tw = font_manager ? font_manager->getTextWidth(feedback_msg.c_str(), FontManager::SMALL) : 0;
            d->drawText((128 - tw) / 2, 35, feedback_msg.c_str(), FontManager::SMALL);
            return;
        }

        if (editor_state == EditorState::BROWSE_ALBUMS) {
            d->drawText(0, HEADER_Y, "BIBLIOTHEQUE", FontManager::SMALL);
            d->drawLine(0, 9, 127, 9);

            std::lock_guard<std::mutex> lk(data_mtx);
            if (albums.empty()) {
                d->drawText(10, 35, "Aucun album", FontManager::SMALL);
                return;
            }
            for (int i = 0; i < VISIBLE_LINES; i++) {
                int idx = album_scroll + i;
                if (idx >= (int)albums.size()) break;
                drawSelectionRow(d, i, albums[idx], idx == album_cursor);
            }
            drawScrollbar(d, (int)albums.size(), VISIBLE_LINES, album_scroll);
        }
        else if (editor_state == EditorState::BROWSE_TRACKS) {
            // En-tête = nom de l'album
            d->drawText(0, HEADER_Y, truncatePx(selected_album, 118).c_str(), FontManager::SMALL);
            d->drawLine(0, 9, 127, 9);

            if (loading) {
                d->drawText(25, 35, "Chargement...", FontManager::SMALL);
                return;
            }

            std::lock_guard<std::mutex> lk(data_mtx);
            // Ligne 0 = [> Jouer album]  Ligne 1 = [+ Ajouter album]  Lignes 2+ = pistes
            int total = 2 + (int)tracks.size();
            for (int i = 0; i < VISIBLE_LINES; i++) {
                int idx = track_scroll + i;
                if (idx >= total) break;
                std::string lbl;
                if      (idx == 0) lbl = "> Jouer album";
                else if (idx == 1) lbl = "+ Ajouter album";
                else {
                    auto& t = tracks[idx - 2];
                    lbl = t.track_num.empty()
                        ? t.title
                        : t.track_num + ". " + t.title;
                }
                drawSelectionRow(d, i, lbl, idx == track_cursor);
            }
            drawScrollbar(d, total, VISIBLE_LINES, track_scroll);
        }
    }

    // -------- rendu écran droit (infos + mini spectre) --------

    void renderInfo(Display* d, AudioProcessor& audio) {
        d->clear();

        // Titre + artiste en cours
        std::string t = mpd_client ? mpd_client->getTitle()  : "";
        std::string a = mpd_client ? mpd_client->getArtist() : "";
        d->drawText(1,  7, truncatePx(t, 126).c_str(), FontManager::SMALL);
        d->drawText(1, 16, truncatePx(a, 126).c_str(), FontManager::SMALL);
        d->drawLine(0, 18, 127, 18);

        // Mini spectrogramme (64 bandes, moitié haute et moitié basse)
        std::array<int, 64> lsp, rsp;
        audio.getLargeSpectrumData(lsp, rsp);
        for (int i = 0; i < 64; i++) {
            int x  = i * 2;
            int hl = 1 + (lsp[i] * 20) / 255;
            int hr = 1 + (rsp[i] * 20) / 255;
            d->drawRect(x, 40 - hl, 1, hl, true); // Canal gauche (haut)
            d->drawRect(x, 41,      1, hr, true); // Canal droit  (bas)
        }
        d->drawLine(0, 40, 127, 40);

        // Aide contextuelle (bas d'écran)
        if (editor_state == EditorState::BROWSE_ALBUMS)
            d->drawText(0, 64, "SEL:entrer BCK:quitter", FontManager::SMALL);
        else
            d->drawText(0, 64, "SEL:ajouter BCK:retour", FontManager::SMALL);
    }

    public:
    PlaylistEditorViz(Display* l, Display* r, MPDClient* m, BluetoothMonitor* bt, FontManager* f)
        : Visualization(l, r, m, bt, f) {}

    ~PlaylistEditorViz() {
        if (load_thread.joinable()) load_thread.join();
    }

    const char* getName() const override { return "Playlist Editor"; }

    // Appelé quand on entre dans cette visualisation
    void onActivate() {
        if (!albums_loaded && !loading)
            startLoadAlbums();
        prev_sel = false;
        prev_back = false;
    }

    // Retourne true si l'éditeur demande à quitter (VIZ_SW depuis la liste d'albums)
    // btn_* = état brut (true = appuyé)
    bool handleInput(bool btn_up, bool btn_dn, bool btn_sel, bool btn_back) {
        // Up/Down are pre-processed (one call per detent), pass through directly
        bool up   = btn_up;
        bool dn   = btn_dn;
        bool sel  = btn_sel;
        bool back = btn_back;

        bool want_exit = false;

        // Feedback : n'importe quelle touche efface le message
        if (editor_state == EditorState::FEEDBACK) {
            if (sel || back) editor_state = EditorState::BROWSE_ALBUMS;
            return false;
        }

        if (editor_state == EditorState::BROWSE_ALBUMS) {
            std::lock_guard<std::mutex> lk(data_mtx);
            int n = (int)albums.size();
            if (back)  { want_exit = true; }
            if (!loading && n > 0) {
                if (dn) {
                    album_cursor = std::min(album_cursor + 1, n - 1);
                    if (album_cursor >= album_scroll + VISIBLE_LINES)
                        album_scroll = album_cursor - VISIBLE_LINES + 1;
                }
                if (up) {
                    album_cursor = std::max(album_cursor - 1, 0);
                    if (album_cursor < album_scroll)
                        album_scroll = album_cursor;
                }
                if (sel && album_cursor < n) {
                    selected_album = albums[album_cursor];
                    track_cursor = 0;
                    track_scroll = 0;
                    editor_state = EditorState::BROWSE_TRACKS;
                    startLoadTracks(selected_album);
                }
            }
        }
        else if (editor_state == EditorState::BROWSE_TRACKS) {
            if (back) {
                editor_state = EditorState::BROWSE_ALBUMS;
                return false;
            }
            if (loading) return false;

            std::lock_guard<std::mutex> lk(data_mtx);
            int total = 2 + (int)tracks.size();
            if (dn) {
                track_cursor = std::min(track_cursor + 1, total - 1);
                if (track_cursor >= track_scroll + VISIBLE_LINES)
                    track_scroll = track_cursor - VISIBLE_LINES + 1;
            }
            if (up) {
                track_cursor = std::max(track_cursor - 1, 0);
                if (track_cursor < track_scroll)
                    track_scroll = track_cursor;
            }
            if (sel) {
                if (track_cursor == 0) {
                    mpd_client->clearAndPlayAlbum(selected_album);
                    setFeedback("  Lecture !");
                } else if (track_cursor == 1) {
                    mpd_client->addAlbumToQueue(selected_album);
                    setFeedback("  Album ajoute !");
                } else {
                    int ti = track_cursor - 2;
                    if (ti < (int)tracks.size()) {
                        mpd_client->addTrackToQueue(tracks[ti].file);
                        setFeedback("  Piste ajoutee !");
                    }
                }
            }
        }
        return want_exit;
    }

    void render(ControlState& cs, AudioProcessor& audio) override {
        renderBrowser(left_display);
        renderInfo(right_display, audio);
        // NE PAS appeler display() ici — le main loop s'en charge
    }
};

// Rotary Encoder 
class RotaryEncoder {
    int clk_pin_, dt_pin_, sw_pin_;
 
    // Gray-code state machine (accessed only from callback context)
    uint8_t state_ = 0;
    int     accumulator_ = 0;
    int     detent_threshold_ = 4; // 4 transitions = 1 detent on KY-040
 
    // Results (written by callbacks, read by main thread)
    std::atomic<int>  rotation_{0};
    std::atomic<bool> button_pressed_{false};
 
    // Button debounce tracking
    uint64_t btn_last_ts_ = 0;
    static constexpr uint64_t BTN_DEBOUNCE_NS = 50000000ULL; // 50 ms
 
    // Transition table: oldState(2 bits) << 2 | newState(2 bits)
    static constexpr int8_t TRANS_[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
 
    // --- Callbacks ---
 
    static void rotaryCB(int num_alerts, lgGpioAlert_p alerts, void* user) {
        auto* self = static_cast<RotaryEncoder*>(user);
        for (int i = 0; i < num_alerts; i++) {
            self->onRotaryEdge();
        }
    }
 
    static void buttonCB(int num_alerts, lgGpioAlert_p alerts, void* user) {
        auto* self = static_cast<RotaryEncoder*>(user);
        for (int i = 0; i < num_alerts; i++) {
            // Falling edge = button pressed (active low)
            if (alerts[i].report.level == 0) {
                uint64_t ts = alerts[i].report.timestamp;
                if ((ts - self->btn_last_ts_) > BTN_DEBOUNCE_NS) {
                    self->btn_last_ts_ = ts;
                    self->button_pressed_.store(true, std::memory_order_relaxed);
                }
            }
        }
    }
 
    void onRotaryEdge() {
        uint8_t cur = static_cast<uint8_t>(
            (lgGpioRead(LGpio::gpio_handle, clk_pin_) << 1) |
             lgGpioRead(LGpio::gpio_handle, dt_pin_));
 
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
    // detent_threshold: Gray-code transitions per detent click.
    //   KY-040 = 4 (default). Some encoders use 2 or 1.
    RotaryEncoder(int clk, int dt, int sw, int detent_threshold = 4)
        : clk_pin_(clk), dt_pin_(dt), sw_pin_(sw),
          detent_threshold_(detent_threshold) {}
 
    // Call after LGpio::init(). Pins must NOT already be claimed
    // (remove LGpio::claimInputPullUp calls for these 3 pins from setupGPIO).
        void init() {
            int h = LGpio::gpio_handle;

            // Claim pins as ALERTS (not just input) — this enables edge notifications
            lgGpioClaimAlert(h, LG_SET_PULL_UP, LG_BOTH_EDGES, clk_pin_, -1);
            lgGpioClaimAlert(h, LG_SET_PULL_UP, LG_BOTH_EDGES, dt_pin_,  -1);
            lgGpioClaimAlert(h, LG_SET_PULL_UP, LG_BOTH_EDGES, sw_pin_,  -1);

            // Debounce
            lgGpioSetDebounce(h, clk_pin_, 5);
            lgGpioSetDebounce(h, dt_pin_,  5);
            lgGpioSetDebounce(h, sw_pin_,  5000);

            // Read initial state
            state_ = static_cast<uint8_t>(
                (lgGpioRead(h, clk_pin_) << 1) | lgGpioRead(h, dt_pin_));

            // Now register callbacks
            lgGpioSetAlertsFunc(h, clk_pin_, rotaryCB, this);
            lgGpioSetAlertsFunc(h, dt_pin_,  rotaryCB, this);
            lgGpioSetAlertsFunc(h, sw_pin_,  buttonCB, this);
        }
 
    ~RotaryEncoder() {
        int h = LGpio::gpio_handle;
        if (h >= 0) {
            lgGpioSetAlertsFunc(h, clk_pin_, nullptr, nullptr);
            lgGpioSetAlertsFunc(h, dt_pin_,  nullptr, nullptr);
            lgGpioSetAlertsFunc(h, sw_pin_,  nullptr, nullptr);
        }
    }
 
    // --- Public API (same interface as before) ---
 
    // Returns +1 (CW), -1 (CCW), or 0 since last call.
    // If multiple detents accumulated, returns the full count.
    int pollRotation() {
        return rotation_.exchange(0, std::memory_order_relaxed);
    }
 
    // Returns true if button was pressed since last call.
    bool pollButton() {
        return button_pressed_.exchange(false, std::memory_order_relaxed);
    }
};
 
constexpr int8_t RotaryEncoder::TRANS_[16];

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
    RotaryEncoder rotary{Config::ROT_CLK, Config::ROT_DT, Config::ROT_SW};
    bool prev_viz_sw = false;  // AJOUT — edge detection VIZ_SW
      
    void onBluetoothConnectionChange(bool connected) {
        if (connected) {
            printf("Bluetooth connected - pausing MPD\n");
            mpd->playPause();   // Remplace system("mpc pause")
        } else {
            printf("Bluetooth disconnected\n");
        }
    }

    // volume overlay
    int  overlay_volume   = -1;
    std::chrono::steady_clock::time_point overlay_ts;
    static constexpr int OVERLAY_DURATION_MS = 1500;

    void drawVolumeOverlay(Display* d) {
        int vol = overlay_volume;
        if (vol < 0) return;

        // Fond de la boite
        d->drawRect(14, 24, 100, 18, false);
        d->drawRect(15, 25,  98, 16, true);  // rempli pour effacer le fond

        // Effacer l'intérieur (inversion : on dessine en noir)
        // Astuce : clear la zone manuellement
        for (int py = 25; py < 41; py++) {
            int row = (py / 8) * Config::SCREEN_WIDTH;
            uint8_t mask = ~(1 << (py % 8));
            for (int px = 15; px < 113; px++)
                d->buffer[row + px] &= mask;
        }
        d->drawRect(14, 24, 100, 18, false);

        // Barre de volume
        int bar_w = (vol * 88) / 100;
        d->drawRect(18, 28, 88,  8, false);
        if (bar_w > 0)
            d->drawRect(18, 28, bar_w, 8, true);

        // Texte pourcentage
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", vol);
        int tw = font_mgr.isInitialized() ?
            font_mgr.getTextWidth(buf, FontManager::SMALL) : 0;
        d->drawText((128 - tw) / 2, 23, buf, FontManager::SMALL);
    }

    void animateCRTBoth(bool power_on) {
        const int total_ms = 400;
        const int phase1_ms = 200;

        uint8_t saved_left[Config::SPI_CHUNK_SIZE];
        uint8_t saved_right[Config::SPI_CHUNK_SIZE];
        memcpy(saved_left, d_left->buffer, Config::SPI_CHUNK_SIZE);
        memcpy(saved_right, d_right->buffer, Config::SPI_CHUNK_SIZE);

        auto start = std::chrono::steady_clock::now();

        while (true) {
            auto now = std::chrono::steady_clock::now();
            int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            if (elapsed >= total_ms) break;

            // Normalize and reverse for power_on
            float t = (float)elapsed / (float)total_ms;
            if (power_on) t = 1.0f - t;

            Display* screens[2] = { d_left.get(), d_right.get() };
            uint8_t* saved[2] = { saved_left, saved_right };

            for (int s = 0; s < 2; s++) {
                auto* d = screens[s];
                memcpy(d->buffer, saved[s], Config::SPI_CHUNK_SIZE);

                float phase_t = (float)elapsed / (float)total_ms;
                if (power_on) phase_t = 1.0f - phase_t;

                if (phase_t < 0.5f) {
                    // Phase 1: vertical collapse
                    float p = phase_t * 2.0f;  // 0→1 within phase
                    int spread = (int)(31.0f * (1.0f - p));
                    int visible_top = 32 - spread;
                    int visible_bot = 32 + spread;

                    for (int py = 0; py < visible_top; py++) {
                        int page = py / 8;
                        uint8_t mask = (1 << (py % 8));
                        for (int px = 0; px < 128; px++)
                            d->buffer[page * 128 + px] &= ~mask;
                    }
                    for (int py = visible_bot + 1; py < 64; py++) {
                        int page = py / 8;
                        uint8_t mask = (1 << (py % 8));
                        for (int px = 0; px < 128; px++)
                            d->buffer[page * 128 + px] &= ~mask;
                    }
                    for (int px = 0; px < 128; px++) {
                        d->drawPixel(px, visible_top);
                        d->drawPixel(px, visible_bot);
                    }
                } else {
                    // Phase 2: horizontal shrink to point
                    float p = (phase_t - 0.5f) * 2.0f;  // 0→1 within phase
                    int half_w = (int)(64.0f * (1.0f - p));
                    int x_start = 64 - half_w;
                    int x_end = 64 + half_w;

                    for (int py = 0; py < 64; py++) {
                        int page = py / 8;
                        uint8_t mask = (1 << (py % 8));
                        for (int px = 0; px < 128; px++) {
                            if (py == 32 && px >= x_start && px <= x_end)
                                continue;
                            d->buffer[page * 128 + px] &= ~mask;
                        }
                    }
                    for (int px = x_start; px <= x_end; px++)
                        d->drawPixel(px, 32);
                }

                d->display();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        d_left->clear();  d_left->display();
        d_right->clear(); d_right->display();
    }

    public:
    VisualizerApp() {
        // Initialize lgpio (replaces bcm2835_init + bcm2835_spi_begin)
        if (!LGpio::init()) {
            throw std::runtime_error("lgpio Init Failed");
        }
        
        d_left = std::make_unique<Display>(Config::LEFT_CS, Config::LEFT_DC, Config::LEFT_RST);
        d_right = std::make_unique<Display>(Config::RIGHT_CS, Config::RIGHT_DC, Config::RIGHT_RST);
        
        d_left->begin(); d_right->begin();
        
        // Load fonts
        const char* fonts[] = {"trixel-square.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", nullptr};
        for(int i=0; fonts[i]; i++) {
            if(font_mgr.init(fonts[i])) {
                d_left->setFont(&font_mgr); d_right->setFont(&font_mgr);
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
        
        // Add all visualizations in order
        vizs.push_back(std::make_unique<VUMeterViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<SpectrumViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<EmptySpectrumViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        //vizs.push_back(std::make_unique<TeubSpectrumViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<WaveformViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        //vizs.push_back(std::make_unique<Grid3DViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        //vizs.push_back(std::make_unique<Particles3DViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<TrackInfoViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        vizs.push_back(std::make_unique<PlaylistEditorViz>(d_left.get(), d_right.get(), mpd.get(), bluetooth.get(), &font_mgr));
        
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
        uint8_t ins[] = {
            Config::VIZ_SW,    // GPIO27 — cycle visualisation
            Config::PLAY_SW,   // GPIO26
            Config::FW_SW,     // GPIO9
            Config::RW_SW      // GPIO6
        };
        for (auto p : ins) LGpio::claimInputPullUp(p);
        LGpio::claimOutput(Config::POWER_LED, 1);
        rotary.init();
    }
    
    void pollControls() {
        int  rot     = rotary.pollRotation();
        bool rot_btn = rotary.pollButton();

        // Edge detection VIZ_SW (niveau bas = appuyé)
        bool viz_raw     = (LGpio::digitalRead(Config::VIZ_SW) == 0);
        bool viz_pressed = viz_raw && !prev_viz_sw;
        prev_viz_sw      = viz_raw;

        // --- Wake on any input ---
        bool any_button = viz_pressed || rot_btn || (rot != 0)
                    || (LGpio::digitalRead(Config::PLAY_SW) == 0)
                    || (LGpio::digitalRead(Config::FW_SW) == 0)
                    || (LGpio::digitalRead(Config::RW_SW) == 0);
        if (any_button) {
            audio->resetSleepTimer();
        }

        auto* editor = dynamic_cast<PlaylistEditorViz*>(vizs[state.current_viz].get());

        if (editor) {
            // ===================== MODE ÉDITEUR =====================
            bool want_exit = false;

            // Feed one event per detent for smooth scrolling
            for (int i = 0; i < abs(rot); i++) {
                want_exit |= editor->handleInput(
                    /*up*/   rot < 0,
                    /*dn*/   rot > 0,
                    /*sel*/  false,
                    /*back*/ false
                );
            }

            // Button events separately (only once)
            if (rot_btn || viz_pressed) {
                want_exit |= editor->handleInput(
                    /*up*/   false,
                    /*dn*/   false,
                    /*sel*/  rot_btn,
                    /*back*/ viz_pressed
                );
            }

            if (want_exit) {
                int n = (int)vizs.size();
                state.current_viz = (state.current_viz + 1) % n;
                if (dynamic_cast<PlaylistEditorViz*>(vizs[state.current_viz].get()))
                    state.current_viz = (state.current_viz + 1) % n;
                printf("Sortie editeur -> %s\n", vizs[state.current_viz]->getName());
            }

        } else {
            // ===================== MODE NORMAL =====================
            // VIZ_SW  → cycle visualisations (y compris vers l'éditeur)
            // ROT_SW  → raccourci direct vers l'éditeur
            // rotation → rien (réservé à la navigation dans le menu)
            // PLAY_SW → play/pause
            // FW_SW   → piste suivante
            // RW_SW   → piste précédente

            if (viz_pressed) {
                int n = (int)vizs.size();
                state.current_viz = (state.current_viz + 1) % n;
                // Sauter l'éditeur dans le cycle VIZ_SW
                if (dynamic_cast<PlaylistEditorViz*>(vizs[state.current_viz].get()))
                    state.current_viz = (state.current_viz + 1) % n;
                printf("Viz: %s\n", vizs[state.current_viz]->getName());
            }

            if (rot != 0) {
                mpd->setVolume(rot * 2); // +/-2% par cran
                overlay_volume = mpd->getVolume();
                overlay_ts     = std::chrono::steady_clock::now();
            }

            if (rot_btn) {
                // Raccourci direct vers l'éditeur depuis n'importe quelle viz
                for (int i = 0; i < (int)vizs.size(); i++) {
                    auto* ed = dynamic_cast<PlaylistEditorViz*>(vizs[i].get());
                    if (ed) {
                        state.current_viz = i;
                        ed->onActivate();
                        printf("Ouverture editeur (ROT_SW)\n");
                        break;
                    }
                }
            }

            if (LGpio::digitalRead(Config::PLAY_SW) == 0) {
                mpd->playPause();
                LGpio::delay(200);
            }
            if (LGpio::digitalRead(Config::FW_SW) == 0) {
                mpd->nextTrack();
                LGpio::delay(200);
            }
            if (LGpio::digitalRead(Config::RW_SW) == 0) {
                mpd->prevTrack();
                LGpio::delay(200);
            }
        }
    }

    void run() {
            if(!audio->start()) return;
        
        using namespace std::chrono;
        auto next_frame = steady_clock::now();
            auto last_user_input = steady_clock::now();
            uint32_t auto_cycle_frames = 0;
            constexpr uint32_t CYCLE_INTERVAL = 36000;  // 10 min @ 60fps
        
        while(state.running && !g_shutdown_req) { 
            pollControls();
                // Détecter interaction utilisateur (bouton viz)
                static int prev_viz = state.current_viz;
                if (state.current_viz != prev_viz) {
                    last_user_input = steady_clock::now();
                    auto_cycle_frames = 0;  // Reset du compteur
                    prev_viz = state.current_viz;
                }
            bool has_audio = audio->checkForAudio();
            
            if(!has_audio && !state.is_sleeping) {
                state.is_sleeping = true;
                audio->setSleepState(true);
                animateCRTBoth(false);
                d_left->sleep(); d_right->sleep();
                LGpio::digitalWrite(Config::POWER_LED, 0);
            } else if (has_audio && state.is_sleeping) {
                state.is_sleeping = false;
                audio->setSleepState(false);
                d_left->wake(); d_right->wake();
                LGpio::digitalWrite(Config::POWER_LED, 1);
                // Render one frame so the buffers have content to reveal
                vizs[state.current_viz]->render(state, *audio);
                animateCRTBoth(true);
            }
            
            if (!state.is_sleeping) {
                bool in_editor = dynamic_cast<PlaylistEditorViz*>(  // ← déclaration unique ici
                    vizs[state.current_viz].get()) != nullptr;

                // Auto-cycle
                if (!in_editor) {
                    auto since_input = duration_cast<seconds>(
                        steady_clock::now() - last_user_input).count();
                    if (since_input > 30) {
                        auto_cycle_frames++;
                        if (auto_cycle_frames >= CYCLE_INTERVAL) {
                            int n = (int)vizs.size();
                            state.current_viz = (state.current_viz + 1) % n;
                            if (dynamic_cast<PlaylistEditorViz*>(vizs[state.current_viz].get()))
                                state.current_viz = (state.current_viz + 1) % n;
                            printf("Auto-cycle to: %s\n", vizs[state.current_viz]->getName());
                            auto_cycle_frames = 0;
                        }
                    }
                } else {
                    auto_cycle_frames = 0;
                }

                vizs[state.current_viz]->render(state, *audio);

                // Overlay volume
                if (!in_editor && overlay_volume >= 0) {  // ← réutilisation, pas de bool
                    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - overlay_ts).count();
                    if (elapsed_ms < OVERLAY_DURATION_MS) {
                        drawVolumeOverlay(d_left.get());
                        drawVolumeOverlay(d_right.get());
                    } else {
                        overlay_volume = -1;
                    }
                }

                d_left->display();
                d_right->display();
                auto now = steady_clock::now();
                if (now > next_frame) {
                    next_frame = now;
                }
                next_frame += milliseconds(16); // Frame limiting (~60FPS)
                std::this_thread::sleep_until(next_frame);
            } else {
                std::this_thread::sleep_for(milliseconds(100));
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
