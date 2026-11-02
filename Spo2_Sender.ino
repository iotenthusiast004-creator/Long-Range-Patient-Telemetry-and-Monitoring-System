/*
  SENDER (connected to laptop via USB Serial Monitor)
  ESP32 + E32-900T20D (LoRa module)

  Wiring:
    E32 VCC  -> ESP32 3.3V
    E32 GND  -> ESP32 GND
    E32 TXD  -> ESP32 GPIO16  (RX2)
    E32 RXD  -> ESP32 GPIO17  (TX2)
    E32 M0   -> GND   (Mode 0: transparent transmission)
    E32 M1   -> GND   (Mode 0: transparent transmission)
    E32 AUX  -> not connected (optional)

  Usage:
    Open Serial Monitor at 115200 baud, line ending "Newline".
    Type BLINK  and press Enter -> remote LED blinks 5 times, replies DONE.
    Type SPO2   and press Enter -> remote MAX30105 is read on demand,
                                   replies VITALS:BPM=xx.x,SPO2=yy.y
                                   (or VITALS:NOFINGER / VITALS:ERROR)
    Type TEMP   and press Enter -> remote DS18B20 is read,
                                   replies TEMP:xx.x (Celsius)
                                   (or TEMP:ERROR)
    Type LIVE   and press Enter -> remote streams BPM, SpO2 and temperature
                                   together every 2 s until you type STOP
    Type STOP   and press Enter -> ends the live stream

  LIVE DASHBOARD (no Serial Monitor spam):
    The sender creates its own Wi-Fi network (no router / internet needed):
        SSID     : SpO2-Monitor
        Password : 12345678
    Join it from a phone or laptop and open  http://192.168.4.1
    The page shows BPM, SpO2 and temperature as big numbers that update in
    place, plus a continuously scrolling graph of each. Use its Start / Stop
    buttons (or type LIVE / STOP in the Serial Monitor) to control streaming.
    Live samples are ALSO printed to the Serial Monitor (set
    PRINT_LIVE_TO_SERIAL to false below to turn that off).

  NOTE: This version is non-blocking. Sending a command doesn't freeze
  the loop while waiting for a reply — everything is handled via polling
  so the Serial Monitor stays responsive either way.
*/

#include <WiFi.h>
#include <WebServer.h>

#define E32_RX_PIN 16    // ESP32 RX2 <- E32 TXD
#define E32_TX_PIN 17    // ESP32 TX2 -> E32 RXD
#define E32_BAUD   9600  // E32 default baud rate (transparent mode)

HardwareSerial LoRaSerial(2);

// ---- Wi-Fi dashboard ----
const char* AP_SSID = "SpO2-Monitor";
const char* AP_PASS = "12345678";          // must be at least 8 characters
const bool  PRINT_LIVE_TO_SERIAL = true;   // true = also print every LIVE sample

WebServer server(80);

float lastBpm  = NAN;       // latest values from the receiver (NAN = no reading)
float lastSpo2 = NAN;
float lastTemp = NAN;
uint32_t liveSeq = 0;       // increments on every LIVE packet (lets the page spot new data)
unsigned long lastLiveDataAt = 0;
bool liveAnnounced = false;

bool waitingForReply = false;
bool liveMode = false;
unsigned long lastLiveRxAt = 0;
bool liveSilenceWarned = false;
const unsigned long LIVE_SILENCE_MS = 10000;  // warn if stream goes quiet
unsigned long commandSentAt = 0;
const unsigned long RESPONSE_TIMEOUT_MS = 15000;  // SpO2 needs a bit more time to settle

// ========================================================================
// DASHBOARD PAGE (self-contained: no internet / CDN needed)
// ========================================================================
const char PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SpO2 Live Monitor</title>
<style>
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,Arial,sans-serif;background:#10141a;color:#e8edf2;padding:14px}
h1{font-size:18px;margin:0 0 10px;font-weight:600}
.bar{display:flex;gap:8px;align-items:center;margin-bottom:12px;flex-wrap:wrap}
button{background:#2a3441;color:#e8edf2;border:0;border-radius:8px;padding:9px 16px;font-size:14px;cursor:pointer}
button:active{background:#3a4658}
#st{margin-left:auto;font-size:13px;padding:5px 10px;border-radius:12px;background:#2a3441}
.card{background:#18202a;border-radius:12px;padding:12px 14px;margin-bottom:12px}
.row{display:flex;align-items:baseline;gap:8px}
.lbl{font-size:13px;color:#9aa7b5;flex:1}
.val{font-size:44px;font-weight:700;line-height:1}
.unit{font-size:16px;color:#9aa7b5}
canvas{width:100%;height:130px;display:block;margin-top:6px}
</style></head><body>
<h1>Live Vitals</h1>
<div class="bar">
  <button onclick="cmd('LIVE')">Start</button>
  <button onclick="cmd('STOP')">Stop</button>
  <span id="st">connecting...</span>
</div>
<div class="card"><div class="row"><span class="lbl">Heart rate</span><span class="val" id="vb" style="color:#ff5d6c">--</span><span class="unit">BPM</span></div><canvas id="cb"></canvas></div>
<div class="card"><div class="row"><span class="lbl">SpO2</span><span class="val" id="vs" style="color:#4da3ff">--</span><span class="unit">%</span></div><canvas id="cs"></canvas></div>
<div class="card"><div class="row"><span class="lbl">Temperature</span><span class="val" id="vt" style="color:#ffb14d">--</span><span class="unit">&deg;C</span></div><canvas id="ct"></canvas></div>
<script>
const MAX=120;                       // points kept on screen (~4 min at 2 s/sample)
const H={b:[],s:[],t:[]};
let lastSeq=-1;
const $=id=>document.getElementById(id);

function cmd(c){fetch('/cmd?c='+c).catch(()=>{});}

function draw(cv,data,color,minSpan,dec){
  const dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  cv.width=w*dpr;cv.height=h*dpr;
  const x=cv.getContext('2d');x.scale(dpr,dpr);x.clearRect(0,0,w,h);
  x.font='11px system-ui';x.textBaseline='middle';
  const v=data.filter(a=>a!==null);
  if(!v.length){x.fillStyle='#6b7785';x.fillText('waiting for data...',8,h/2);return;}
  let lo=Math.min(...v),hi=Math.max(...v);
  if(hi-lo<minSpan){const m=(hi+lo)/2;lo=m-minSpan/2;hi=m+minSpan/2;}
  const pad=(hi-lo)*0.12;lo-=pad;hi+=pad;
  const L=34,R=6,T=6,B=6,pw=w-L-R,ph=h-T-B;
  const Y=a=>T+ph-(a-lo)/(hi-lo)*ph;
  x.strokeStyle='#26303c';x.fillStyle='#6b7785';x.lineWidth=1;
  for(let i=0;i<=2;i++){const g=lo+(hi-lo)*i/2,yy=Y(g);
    x.beginPath();x.moveTo(L,yy);x.lineTo(w-R,yy);x.stroke();
    x.fillText(g.toFixed(dec),2,yy);}
  x.strokeStyle=color;x.lineWidth=2;x.lineJoin='round';
  let pen=false;x.beginPath();
  data.forEach((a,i)=>{
    const px=L+(MAX-data.length+i)/(MAX-1)*pw;
    if(a===null){pen=false;return;}
    if(!pen){x.moveTo(px,Y(a));pen=true;}else x.lineTo(px,Y(a));
  });
  x.stroke();
}
function render(){
  draw($('cb'),H.b,'#ff5d6c',20,0);
  draw($('cs'),H.s,'#4da3ff',4,0);
  draw($('ct'),H.t,'#ffb14d',1,1);
}
function push(a,v){a.push(v);if(a.length>MAX)a.shift();}
function show(id,v,dec){$(id).textContent=(v===null)?'--':v.toFixed(dec);}

async function poll(){
  try{
    const d=await (await fetch('/data',{cache:'no-store'})).json();
    show('vb',d.bpm,0);show('vs',d.spo2,0);show('vt',d.temp,1);
    if(d.seq!==lastSeq){
      lastSeq=d.seq;
      if(d.seq>0){push(H.b,d.bpm);push(H.s,d.spo2);push(H.t,d.temp);render();}
    }
    const st=$('st');
    if(!d.live){st.textContent='Stopped';st.style.background='#2a3441';}
    else if(d.age<0||d.age>8000){st.textContent='No data';st.style.background='#7a5a1f';}
    else{st.textContent='Streaming';st.style.background='#1f6b3a';}
  }catch(e){const st=$('st');st.textContent='Disconnected';st.style.background='#7a2a2a';}
}
window.addEventListener('resize',render);
setInterval(poll,1000);poll();render();
</script></body></html>)rawliteral";

String jsonNum(float v) {
  return isnan(v) ? String("null") : String(v, 1);
}

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

void handleData() {
  long age = (lastLiveDataAt == 0) ? -1 : (long)(millis() - lastLiveDataAt);
  String j = "{\"live\":";
  j += liveMode ? "true" : "false";
  j += ",\"seq\":";   j += liveSeq;
  j += ",\"age\":";   j += age;
  j += ",\"bpm\":";   j += jsonNum(lastBpm);
  j += ",\"spo2\":";  j += jsonNum(lastSpo2);
  j += ",\"temp\":";  j += jsonNum(lastTemp);
  j += "}";
  server.send(200, "application/json", j);
}

void handleCmd() {
  String c = server.arg("c");
  c.toUpperCase();
  if (c == "LIVE" || c == "STOP") {   // only streaming control from the web page
    sendCommand(c);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad command");
  }
}

// Converts "NA" / text into a float (NAN when there is no reading)
float parseValue(const String &s) {
  if (s == "NA" || s.length() == 0) return NAN;
  return s.toFloat();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  LoRaSerial.begin(E32_BAUD, SERIAL_8N1, E32_RX_PIN, E32_TX_PIN);

  // Wi-Fi access point + dashboard
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cmd", handleCmd);
  server.begin();

  Serial.println();
  Serial.println("=== LoRa Sender Ready (BPM + SpO2) ===");
  Serial.println("Type BLINK to blink the remote LED");
  Serial.println("Type SPO2  to request BPM and SpO2 from the remote sensor");
  Serial.println("Type TEMP  to request temperature from the remote DS18B20");
  Serial.println("Type LIVE  to stream BPM + SpO2 + temperature in real time");
  Serial.println("Type STOP  to end the live stream");
  Serial.println();
  Serial.print("Dashboard: join Wi-Fi \"");
  Serial.print(AP_SSID);
  Serial.print("\" (password ");
  Serial.print(AP_PASS);
  Serial.print(") and open http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();
  handleUserInput();
  handleIncomingData();
  checkResponseTimeout();
  checkLiveSilence();
}

// Returns the value after "KEY=" up to the next comma (or end of string)
String getField(const String &payload, const String &key) {
  int i = payload.indexOf(key);
  if (i < 0) return "NA";
  i += key.length();
  int j = payload.indexOf(',', i);
  return (j < 0) ? payload.substring(i) : payload.substring(i, j);
}

// Sends a command over LoRa and sets up reply tracking
void sendCommand(const String &cmd) {
  // Discard stale bytes so an old leftover reply isn't misread as new
  while (LoRaSerial.available()) {
    LoRaSerial.read();
  }

  LoRaSerial.println(cmd);
  Serial.print("[TX] Sent command: ");
  Serial.println(cmd);

  if (cmd == "LIVE") {
    liveMode = true;
    lastLiveRxAt = millis();
    liveSilenceWarned = false;
    liveAnnounced = false;
    waitingForReply = false;   // a stream has no single reply to wait for
    Serial.println("[LIVE] Waiting for data... open the dashboard, type STOP to end.");
  } else {
    waitingForReply = true;    // STOP expects a "STOPPED" reply
    commandSentAt = millis();
  }
}

// Reads a command typed into the Serial Monitor and sends it over LoRa
void handleUserInput() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.length() > 0) {
      sendCommand(cmd);
    }
  }
}

// Reads anything arriving over LoRa: a DONE reply, VITALS or TEMP reading
void handleIncomingData() {
  if (LoRaSerial.available()) {
    String data = LoRaSerial.readStringUntil('\n');
    data.trim();

    if (data.length() == 0) {
      return;
    }

    if (data == "DONE") {
      Serial.println("[RX] Feedback received: Remote LED blinked successfully!");
      Serial.println("------------------------------------------");
      waitingForReply = false;

    } else if (data.startsWith("VITALS:")) {
      String payload = data.substring(7);  // strip "VITALS:"

      if (payload == "NOFINGER") {
        Serial.println("[SENSOR] No finger detected on remote sensor.");
      } else if (payload == "ERROR") {
        Serial.println("[SENSOR] Remote reported: MAX30105 error / not ready.");
      } else {
        // Expected format: BPM=72.5,SPO2=98.0
        Serial.print("[SENSOR] ");
        Serial.println(payload);

        // Optional friendly parse
        int bpmIdx = payload.indexOf("BPM=");
        int spo2Idx = payload.indexOf("SPO2=");
        if (bpmIdx >= 0 && spo2Idx > bpmIdx) {
          String bpmStr  = payload.substring(bpmIdx + 4, payload.indexOf(',', bpmIdx));
          String spo2Str = payload.substring(spo2Idx + 5);
          Serial.print("         Heart Rate : ");
          Serial.print(bpmStr);
          Serial.println(" BPM");
          Serial.print("         SpO2       : ");
          Serial.print(spo2Str);
          Serial.println(" %");
        }
      }
      Serial.println("------------------------------------------");
      waitingForReply = false;

    } else if (data.startsWith("LIVE:")) {
      if (!liveMode) return;   // late packet after STOP, ignore
      String payload = data.substring(5);
      lastLiveRxAt = millis();
      liveSilenceWarned = false;

      // Store the values for the web dashboard
      lastBpm  = parseValue(getField(payload, "BPM="));
      lastSpo2 = parseValue(getField(payload, "SPO2="));
      lastTemp = parseValue(getField(payload, "TEMP="));
      lastLiveDataAt = millis();
      liveSeq++;

      if (!liveAnnounced) {
        liveAnnounced = true;
        Serial.println("[LIVE] Data is flowing - view it on the dashboard.");
      }

      if (PRINT_LIVE_TO_SERIAL) {
        Serial.print("[LIVE] Heart Rate: ");
        Serial.print(getField(payload, "BPM="));
        Serial.print(" BPM | SpO2: ");
        Serial.print(getField(payload, "SPO2="));
        Serial.print(" % | Temp: ");
        Serial.print(getField(payload, "TEMP="));
        Serial.println(" C");
      }

    } else if (data == "STOPPED") {
      liveMode = false;
      waitingForReply = false;
      Serial.println("[LIVE] Stream stopped.");
      Serial.println("------------------------------------------");

    } else if (data.startsWith("TEMP:")) {
      String payload = data.substring(5);  // strip "TEMP:"

      if (payload == "ERROR") {
        Serial.println("[SENSOR] Remote reported: DS18B20 error / not connected.");
      } else {
        float tC = payload.toFloat();
        float tF = tC * 9.0f / 5.0f + 32.0f;
        Serial.print("[SENSOR] Temperature : ");
        Serial.print(tC, 1);
        Serial.print(" C  (");
        Serial.print(tF, 1);
        Serial.println(" F)");
      }
      Serial.println("------------------------------------------");
      waitingForReply = false;

    } else {
      Serial.print("[RX] Unrecognized data: ");
      Serial.println(data);
    }
  }
}

// While streaming, warn once if packets stop arriving
void checkLiveSilence() {
  if (liveMode && !liveSilenceWarned && (millis() - lastLiveRxAt > LIVE_SILENCE_MS)) {
    Serial.println("[LIVE] No data for 10 s. Check power/wiring/range (stream still active).");
    liveSilenceWarned = true;
  }
}

// If we sent a command and haven't heard back within the timeout, report it
void checkResponseTimeout() {
  if (waitingForReply && (millis() - commandSentAt > RESPONSE_TIMEOUT_MS)) {
    Serial.println("[RX] No response received (timeout). Check power/wiring/range/finger placement.");
    Serial.println("------------------------------------------");
    waitingForReply = false;
  }
}