# OpenDartboard API 🎯

Real-time dart scoring via WebSocket and HTTP REST API.

## WebSocket Endpoint (Primary)

```
ws://<ip-adress>:13520/scores
```

### Real-time Score Streaming

The **main feature** - connects and receives live dart scores as JSON messages in real-time.

**Example Score Message:**

```json
{
  "score": "D20",
  "position": { "x": 150, "y": 200 },
  "confidence": 0.95,
  "camera": 0,
  "processing_time": 15,
  "timestamp": 1699123456789
}
```

### Field Contract

| Field             | Type      | Range                                                | Description             |
| ----------------- | --------- | ---------------------------------------------------- | ----------------------- |
| `score`           | `string`  | `"S1"-"D20"`, `"BULL"`, `"OUTER"`, `"MISS"`, `"END"` | Dart score value        |
| `position.x`      | `integer` | `0-XXX`                                              | X coordinate in pixels  |
| `position.y`      | `integer` | `0-XXX`                                              | Y coordinate in pixels  |
| `confidence`      | `float`   | `0.0-1.0`                                            | Detection confidence    |
| `camera`          | `integer` | `0-2`                                                | Camera index            |
| `processing_time` | `integer` | `1-1000`                                             | Processing time in ms   |
| `timestamp`       | `integer` | Unix timestamp                                       | Message timestamp in ms |

## Simple Client Example

### JavaScript

```javascript
const socket = new WebSocket("ws://<ip-adress>:13520/scores");

socket.onmessage = function (event) {
  const score = JSON.parse(event.data);
  console.log(
    `Score: ${score.score} at (${score.position.x}, ${score.position.y})`
  );
};

socket.onopen = () => console.log("Connected to OpenDartboard");
socket.onclose = () => console.log("Disconnected");
```

### Python

```python
import websocket
import json

def on_message(ws, message):
    score = json.loads(message)
    print(f"Score: {score['score']} at ({score['position']['x']}, {score['position']['y']})")

ws = websocket.WebSocketApp("ws://<ip-adress>:13520/scores", on_message=on_message)
ws.run_forever()
```

### iOS (Swift)

```swift
import Foundation
import Starscream

class DartboardClient: WebSocketDelegate {
    var socket: WebSocket!

    init() {
        var request = URLRequest(url: URL(string: "ws://<ip-address>:13520/scores")!)
        socket = WebSocket(request: request)
        socket.delegate = self
        socket.connect()
    }

    func didReceive(event: WebSocketEvent, client: WebSocket) {
        switch event {
        case .text(let text):
            if let data = text.data(using: .utf8),
               let scoreData = try? JSONDecoder().decode(ScoreData.self, from: data) {
                print("Score: \(scoreData.score) at (\(scoreData.position.x), \(scoreData.position.y))")
            }
        case .connected:
            print("Connected to OpenDartboard")
        case .disconnected(let reason, let code):
            print("Disconnected: \(reason)")
        default:
            break
        }
    }
}

struct ScoreData: Codable {
    let score: String
    let position: Position
    let confidence: Double
    let camera: Int
    let processing_time: Int
    let timestamp: Int64
}

struct Position: Codable {
    let x: Int
    let y: Int
}
```

### Android (Kotlin)

```kotlin
import okhttp3.*
import org.json.JSONObject

class DartboardClient : WebSocketListener() {
    private var webSocket: WebSocket? = null

    fun connect() {
        val client = OkHttpClient()
        val request = Request.Builder()
            .url("ws://<ip-address>:13520/scores")
            .build()
        webSocket = client.newWebSocket(request, this)
    }

    override fun onMessage(webSocket: WebSocket, text: String) {
        try {
            val scoreData = JSONObject(text)
            val score = scoreData.getString("score")
            val position = scoreData.getJSONObject("position")
            val x = position.getInt("x")
            val y = position.getInt("y")

            println("Score: $score at ($x, $y)")
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    override fun onOpen(webSocket: WebSocket, response: Response) {
        println("Connected to OpenDartboard")
    }

    override fun onClosed(webSocket: WebSocket, code: Int, reason: String) {
        println("Disconnected: $reason")
    }

    override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
        println("Connection failed: ${t.message}")
    }
}

// Usage
val client = DartboardClient()
client.connect()
```

---

## HTTP REST API (Secondary)

### Health Check

```
GET http://<ip-adress>:13520/health
```

**Response:**

```json
{
  "status": "ok",
  "service": "OpenDartboard"
}
```

### Configuration Management

#### [WIP] Get Current Configuration

```
GET http://<ip-adress>:13520/config
```

**Response:**

```json
{
  "key": "value"
}
```

#### [WIP] Update Configuration

```
PUT http://<ip-adress>:13520/config
```

```json
{
  "key": "value"
}
```

**Response:**

```json
{
  "status": "updated"
}
```

### Calibration Control

#### [WIP] Start Calibration Process

```
POST http://<ip-adress>:13520/calibrate/start
```

**Response:**

```json
{
  "status": "started"
}
```

#### Get Calibration Status

```
GET http://<ip-adress>:13520/calibrate/status
```

**Response:**

```json
{
  "calibrating": true
}
```

## Error Handling

### WebSocket Errors

- **Connection Failed**: Check if OpenDartboard is running
- **Connection Lost**: Implement exponential backoff reconnection
- **Invalid JSON**: Parse errors in client code

### HTTP Errors

- **400 Bad Request**: Invalid JSON in PUT/POST requests
- **404 Not Found**: Endpoint doesn't exist
- **500 Internal Error**: Server-side issue

## Test Clients

- **REST API**: Use cURL commands above or Postman/Insomnia

## Performance Notes

- **WebSocket**: No rate limiting, handle high-frequency dart detections
- **Ping frames**: Sent every 30 seconds for connection keep-alive
- **Message bursts**: Possible during rapid scoring sequences
- **Reconnection**: Client responsibility for WebSocket reconnection
