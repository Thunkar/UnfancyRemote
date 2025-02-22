#include "wifi_setup.h"

const char* accessPointName = "Unfance Remote AP";

DNSServer dnsServer;
AsyncWebServer server(80);
AsyncWebSocket ws("/");

static AsyncCallbackJsonWebHandler *settingsHandler = new AsyncCallbackJsonWebHandler("/settings");
void configureSettingsHandler() {
  settingsHandler->setMethod(HTTP_POST | HTTP_GET);
  settingsHandler->onRequest([](AsyncWebServerRequest *request, JsonVariant &json) {
    if(request->method() == HTTP_POST) {
      unsigned int nCells = json.as<JsonObject>()["nCells"];
      unsigned int txIdentity = json.as<JsonObject>()["txIdentity"];
      unsigned int channel = json.as<JsonObject>()["channel"];
      bool isDual = json.as<JsonObject>()["isDual"];
      config.nCells = nCells;
      config.TXIdentity = txIdentity;
      config.channel = channel;
      config.isDual = isDual;
      writeConfig();
      request->send(200, "text/plain", "Ok");
    } else {
      AsyncJsonResponse *response = new AsyncJsonResponse();
      JsonObject root = response->getRoot().to<JsonObject>();
      root["nCells"] = config.nCells;
      root["txIdentity"] = config.TXIdentity;
      root["channel"] = config.channel;
      root["isDual"] = config.isDual;
      response->setLength();
      request->send(response);
    }
  });

}

class CaptivePortalHandler : public AsyncWebHandler {
public:
  CaptivePortalHandler() {}
  virtual ~CaptivePortalHandler() {}

  bool canHandle(AsyncWebServerRequest *request){
    return request->url() == "/" && request->method() == HTTP_GET;
  }

  void handleRequest(AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", String(), false);
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

  configureSettingsHandler();

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.addHandler(new CaptivePortalHandler()).setFilter(ON_AP_FILTER);
  server.addHandler(settingsHandler);
  server.onNotFound([&](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
  });
  
  server.begin();
}

bool doServerWork(unsigned long now) {
  dnsServer.processNextRequest();
  ws.textAll(
    state.batteryVoltage + String(",") + 
    state.boardVoltage + String(",") + 
    state.rawThrottle1Value + String(",") + 
    state.rawThrottle2Value + String(",")
  );
  return true;
}