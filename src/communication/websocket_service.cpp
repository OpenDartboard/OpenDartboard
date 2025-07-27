#include "websocket_service.hpp"
#include "utils.hpp"
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <set>
#include <mutex>
#include <sstream>
#include <iomanip>

using namespace std;
using json = nlohmann::json;

// Proper SHA1 implementation for WebSocket handshake
class SHA1
{
private:
    uint32_t h[5];
    uint64_t len;
    uint8_t buffer[64];
    uint8_t bufferPos;

    uint32_t leftRotate(uint32_t value, int amount)
    {
        return (value << amount) | (value >> (32 - amount));
    }

    void processBlock()
    {
        uint32_t w[80];

        // Copy buffer to w[0..15]
        for (int i = 0; i < 16; i++)
        {
            w[i] = (buffer[i * 4] << 24) | (buffer[i * 4 + 1] << 16) |
                   (buffer[i * 4 + 2] << 8) | buffer[i * 4 + 3];
        }

        // Extend w[16..79]
        for (int i = 16; i < 80; i++)
        {
            w[i] = leftRotate(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }

        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];

        for (int i = 0; i < 80; i++)
        {
            uint32_t f, k;
            if (i < 20)
            {
                f = (b & c) | ((~b) & d);
                k = 0x5A827999;
            }
            else if (i < 40)
            {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1;
            }
            else if (i < 60)
            {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDC;
            }
            else
            {
                f = b ^ c ^ d;
                k = 0xCA62C1D6;
            }

            uint32_t temp = leftRotate(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = leftRotate(b, 30);
            b = a;
            a = temp;
        }

        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
    }

public:
    SHA1()
    {
        h[0] = 0x67452301;
        h[1] = 0xEFCDAB89;
        h[2] = 0x98BADCFE;
        h[3] = 0x10325476;
        h[4] = 0xC3D2E1F0;
        len = 0;
        bufferPos = 0;
    }

    void update(const uint8_t *data, size_t size)
    {
        for (size_t i = 0; i < size; i++)
        {
            buffer[bufferPos++] = data[i];
            len++;

            if (bufferPos == 64)
            {
                processBlock();
                bufferPos = 0;
            }
        }
    }

    vector<uint8_t> finalize()
    {
        // Padding
        buffer[bufferPos++] = 0x80;

        if (bufferPos > 56)
        {
            while (bufferPos < 64)
                buffer[bufferPos++] = 0;
            processBlock();
            bufferPos = 0;
        }

        while (bufferPos < 56)
            buffer[bufferPos++] = 0;

        // Length in bits
        uint64_t bitLen = len * 8;
        for (int i = 7; i >= 0; i--)
        {
            buffer[56 + i] = bitLen & 0xFF;
            bitLen >>= 8;
        }

        processBlock();

        vector<uint8_t> result(20);
        for (int i = 0; i < 5; i++)
        {
            result[i * 4] = (h[i] >> 24) & 0xFF;
            result[i * 4 + 1] = (h[i] >> 16) & 0xFF;
            result[i * 4 + 2] = (h[i] >> 8) & 0xFF;
            result[i * 4 + 3] = h[i] & 0xFF;
        }

        return result;
    }
};

// Base64 encode
string base64_encode(const vector<uint8_t> &data)
{
    const string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    string result;

    for (size_t i = 0; i < data.size(); i += 3)
    {
        uint32_t tmp = 0;
        int padding = 0;

        for (int j = 0; j < 3; j++)
        {
            tmp <<= 8;
            if (i + j < data.size())
            {
                tmp |= data[i + j];
            }
            else
            {
                padding++;
            }
        }

        for (int j = 0; j < 4; j++)
        {
            if (j < 4 - padding)
            {
                result += chars[(tmp >> (6 * (3 - j))) & 0x3F];
            }
            else
            {
                result += '=';
            }
        }
    }

    return result;
}

// Generate proper WebSocket accept key
string generate_websocket_accept(const string &key)
{
    string combined = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

    SHA1 sha1;
    sha1.update((const uint8_t *)combined.c_str(), combined.length());
    vector<uint8_t> hash = sha1.finalize();

    return base64_encode(hash);
}

// WebSocket frame creation
vector<uint8_t> create_websocket_frame(const string &payload)
{
    vector<uint8_t> frame;

    // FIN + TEXT opcode
    frame.push_back(0x81);

    size_t payload_len = payload.length();
    if (payload_len < 126)
    {
        frame.push_back(payload_len);
    }
    else if (payload_len < 65536)
    {
        frame.push_back(126);
        frame.push_back((payload_len >> 8) & 0xFF);
        frame.push_back(payload_len & 0xFF);
    }
    else
    {
        frame.push_back(127);
        for (int i = 7; i >= 0; i--)
        {
            frame.push_back((payload_len >> (i * 8)) & 0xFF);
        }
    }

    // Add payload
    for (char c : payload)
    {
        frame.push_back(c);
    }

    return frame;
}

// Global active connections and their sinks
struct ActiveConnection
{
    httplib::DataSink *sink; // Store pointer, NOT shared_ptr that tries to copy
    atomic<bool> active{true};

    ActiveConnection(httplib::DataSink *s) : sink(s) {}
};

static vector<shared_ptr<ActiveConnection>> active_connections;
static mutex connections_mutex;

WebSocketService::WebSocketService(shared_ptr<ScoreQueue> queue, int port)
    : score_queue_(queue), port_(port) {}

WebSocketService::~WebSocketService()
{
    stop();
}

void WebSocketService::start()
{
    if (running_)
        return;

    running_ = true;
    worker_thread_ = thread(&WebSocketService::run, this);
    log_debug("WebSocket service starting on port " + to_string(port_));
}

void WebSocketService::stop()
{
    running_ = false;

    // Mark all connections as inactive
    {
        lock_guard<mutex> lock(connections_mutex);
        for (auto &conn : active_connections)
        {
            conn->active = false;
        }
    }

    if (server_)
    {
        server_->stop();
    }
    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }
    log_info("WebSocket service stopped");
}

void WebSocketService::run()
{
    server_ = make_unique<httplib::Server>();

    try
    {
        // CORS headers
        server_->set_default_headers({{"Access-Control-Allow-Origin", "*"},
                                      {"Access-Control-Allow-Methods", "GET, POST, OPTIONS"},
                                      {"Access-Control-Allow-Headers", "Content-Type"}});

        // WebSocket endpoint with REAL streaming
        server_->Get("/scores", [&](const httplib::Request &req, httplib::Response &res)
                     {
            // Check for WebSocket upgrade
            if (req.get_header_value("Upgrade") == "websocket" &&
                req.get_header_value("Connection").find("Upgrade") != string::npos) {
                
                string websocket_key = req.get_header_value("Sec-WebSocket-Key");
                if (websocket_key.empty()) {
                    res.status = 400;
                    return;
                }
                
                // Generate accept key
                string accept_key = generate_websocket_accept(websocket_key);
                
                // Send WebSocket handshake response
                res.status = 101;
                res.set_header("Upgrade", "websocket");
                res.set_header("Connection", "Upgrade");
                res.set_header("Sec-WebSocket-Accept", accept_key);
                
                // Set up streaming response with ACTUAL WebSocket data flow
                res.set_content_provider(
                    "application/octet-stream",
                    [&](size_t offset, httplib::DataSink& sink) -> bool {
                        // Create connection object with POINTER not copy
                        auto connection = make_shared<ActiveConnection>(&sink);
                        
                        // Add to active connections
                        {
                            lock_guard<mutex> lock(connections_mutex);
                            active_connections.push_back(connection);
                        }
                        
                        log_info("WebSocket client connected - STREAMING ACTIVE");
                        
                        // Keep connection alive and send ping frames
                        auto last_ping = chrono::steady_clock::now();
                        
                        while (running_ && connection->active) {
                            auto now = chrono::steady_clock::now();
                            
                            // Send ping every 30 seconds
                            if (chrono::duration_cast<chrono::seconds>(now - last_ping).count() >= 30) {
                                vector<uint8_t> ping_frame = {0x89, 0x00}; // Ping frame
                                try {
                                    if (!sink.write((const char*)ping_frame.data(), ping_frame.size())) {
                                        break; // Connection closed
                                    }
                                    last_ping = now;
                                } catch (...) {
                                    break;
                                }
                            }
                            
                            this_thread::sleep_for(chrono::milliseconds(100));
                        }
                        
                        // Remove from active connections
                        {
                            lock_guard<mutex> lock(connections_mutex);
                            active_connections.erase(
                                remove(active_connections.begin(), active_connections.end(), connection),
                                active_connections.end()
                            );
                        }
                        
                        log_info("WebSocket client disconnected");
                        return false; // End streaming
                    }
                );
                
                return;
            }
            
            // Regular HTTP response
            res.set_content("WebSocket endpoint. Connect with ws://localhost:" + to_string(port_) + "/scores", "text/plain"); });

        // Health check
        server_->Get("/health", [](const httplib::Request &req, httplib::Response &res)
                     { res.set_content("{\"status\":\"ok\",\"service\":\"OpenDartboard\"}", "application/json"); });

        // Configuration endpoints
        server_->Get("/config", [](const httplib::Request &req, httplib::Response &res)
                     {
            json config;
            // TODO: return actual configuration
            config["todo"] = "yes";
            res.set_content(config.dump(), "application/json"); });

        server_->Put("/config", [](const httplib::Request &req, httplib::Response &res)
                     {
            try {
                json new_config = json::parse(req.body);
                // TODO: Apply configuration changes
                log_info("TODO: Configuration updated: " + new_config.dump());
                res.set_content("{\"status\":\"(TODO)updated\"}", "application/json");
            } catch (const exception& e) {
                res.status = 400;
                res.set_content("{\"error\":\"Invalid JSON\"}", "application/json");
            } });

        // Calibration endpoints
        server_->Post("/calibrate/start", [](const httplib::Request &req, httplib::Response &res)
                      {
            // TODO: Start calibration process
            res.set_content("{\"status\":\"(TODO)calibration_started\"}", "application/json"); });

        server_->Get("/calibrate/status", [](const httplib::Request &req, httplib::Response &res)
                     {
            json status;
            // TODO: return actual calibration status
            status["todo"] = "yes";
            res.set_content(status.dump(), "application/json"); });

        server_->Get("/debug/list", [](const httplib::Request &req, httplib::Response &res)
                     {
            json files = json::array();
            
            for (const auto& entry : filesystem::recursive_directory_iterator("debug_frames")) {
                if (entry.is_regular_file() && entry.path().extension() == ".jpg") {
                    string path = entry.path().string();
                    // Remove debug_frames/ prefix - C++17 compatible
                    if (path.substr(0, 13) == "debug_frames/") {
                        path = path.substr(13);
                    }
                    files.push_back(path);
                }
            }
            
            res.set_content(files.dump(), "application/json"); });

        server_->Get("/debug/logs", [](const httplib::Request &req, httplib::Response &res)
                     {
            // Use tail command for fast last N lines
            int result = system("tail -100 debug_frames/opendartboard.log > /tmp/recent_logs.txt 2>/dev/null");
            if (result != 0) {
                res.set_content("[]", "application/json");
                return;
            }
            
            ifstream file("/tmp/recent_logs.txt");
            if (!file) {
                res.set_content("[]", "application/json");
                return;
            }
            
            json logs = json::array();
            string line;
            while (getline(file, line)) {
                if (!line.empty()) {
                    logs.push_back(line);
                }
            }
            
            res.set_content(logs.dump(), "application/json"); });

        server_->Get(R"(/debug/(.+))", [](const httplib::Request &req, httplib::Response &res)
                     {
            string path = "debug_frames/" + req.matches[1].str();
            
            ifstream file(path, ios::binary);
            if (!file) {
                res.status = 404;
                return;
            }
            
            string content((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
            res.set_content(content, "image/jpeg"); });

        server_->Get("/info", [](const httplib::Request &req, httplib::Response &res)
                     {
            ifstream file("cache/info.json");
            if (!file) {
                res.set_content("{\"error\":\"info.json not found\"}", "application/json");
                return;
            }
            
            string content((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
            res.set_content(content, "application/json"); });

        // Start server thread
        thread server_thread([&]()
                             {
            log_info("WebSocket server listening on ws://0.0.0.0:" + to_string(port_) + "/scores");
            log_info("Rest server listening on http://0.0.0.0:" + to_string(port_) + "/");
            server_->listen("0.0.0.0", port_); });

        // MAIN BROADCASTING LOOP - this is where the magic happens!
        while (running_)
        {
            DetectorResult result;
            if (score_queue_->pop(result, 100))
            {
                if (result.dart_detected)
                {
                    string json_message = formatScoreJson(result);
                    broadcastScore(json_message);
                }
            }
        }

        server_->stop();
        if (server_thread.joinable())
        {
            server_thread.join();
        }
    }
    catch (const exception &e)
    {
        log_error("WebSocket server error: " + string(e.what()));
    }
}

void WebSocketService::broadcastScore(const string &json_message)
{
    vector<uint8_t> frame = create_websocket_frame(json_message);

    lock_guard<mutex> lock(connections_mutex);

    // Send to ALL active WebSocket connections
    for (auto it = active_connections.begin(); it != active_connections.end();)
    {
        try
        {
            if ((*it)->active && (*it)->sink)
            {
                bool success = (*it)->sink->write((const char *)frame.data(), frame.size());
                if (!success)
                {
                    (*it)->active = false;
                    it = active_connections.erase(it);
                }
                else
                {
                    ++it;
                }
            }
            else
            {
                it = active_connections.erase(it);
            }
        }
        catch (const exception &e)
        {
            log_error("Error broadcasting to WebSocket client: " + string(e.what()));
            (*it)->active = false;
            it = active_connections.erase(it);
        }
    }

    if (!active_connections.empty())
    {
        log_debug("Broadcasted score '" + json_message + "' to " + to_string(active_connections.size()) + " WebSocket clients");
    }
}

string WebSocketService::formatScoreJson(const DetectorResult &result)
{
    json j;
    j["score"] = result.score;
    j["position"] = {
        {"x", (int)result.position.x},
        {"y", (int)result.position.y}};
    j["confidence"] = result.confidence;
    j["camera"] = result.camera_index;
    j["processing_time"] = result.processing_time_ms;
    j["timestamp"] = chrono::duration_cast<chrono::milliseconds>(
                         chrono::system_clock::now().time_since_epoch())
                         .count();

    return j.dump();
}