#include "wifi_setup.h"

const char* accessPointName = "Unfance Remote AP";

DNSServer dnsServer;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


class CaptivePortalHandler : public AsyncWebHandler {
public:
  CaptivePortalHandler() {}
  virtual ~CaptivePortalHandler() {}

  bool canHandle(AsyncWebServerRequest *request){
    return request->url() == "/" || request->url() == "/settings" || request->url() == "/calibration";
  }

  void handleRequest(AsyncWebServerRequest *request, JsonVariant &json) {
    if (request->url() == "/settings") {
      serializeJson(json, Serial);
      unsigned int value = json.as<unsigned int>()["calCenter"];
      Serial.println(value);
      request->send(200, "text/plain", "Ok");
    } else if (request->url() == "/calibration") {

    } else {
      request->send(SPIFFS, "/index.html", String(), false);
    }
  }
};


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    Serial.println(String((char*)data));
    if (strcmp((char*)data, "save") == 0) {
      config.centerAcc = state.rawThrottle1Value;
      writeConfig();
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void setupServer(){
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(300);
  dnsServer.start(53, "*", WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.addHandler(new CaptivePortalHandler()).setFilter(ON_AP_FILTER);
  server.onNotFound([&](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
  });
  
  server.begin();
}

bool doServerWork(unsigned long now) {
  dnsServer.processNextRequest();
  ws.textAll(
    config.channel + String(",") + 
    config.TXIdentity + String(",") + 
    state.batteryVoltage + String(",") + 
    config.nCells + String(",") + 
    state.boardVoltage + String(",") + 
    state.rawThrottle1Value + String(",") + 
    state.rawThrottle2Value + String(",") + 
    config.calBrake + String(",") + 
    config.calAcc + String(",") + 
    config.centerAcc + String(",") + 
    config.centerBrake + String(",") + 
    config.inverted + String(",") +
    config.isDual
  );
  return true;
}