#include "wifi_setup.h"

DNSServer dnsServer;
AsyncWebServer server(80);
AsyncWebSocket ws("/");

static AsyncCallbackJsonWebHandler *settingsHandler = new AsyncCallbackJsonWebHandler("/settings");
void configureSettingsHandler() {
  settingsHandler->setMethod(HTTP_POST | HTTP_GET);
  settingsHandler->onRequest([](AsyncWebServerRequest *request, JsonVariant &json) {
    if(request->method() == HTTP_POST) {
      unsigned int cellN = json.as<JsonObject>()["cellN"];
      unsigned int identity = json.as<JsonObject>()["identity"];
      unsigned int channel = json.as<JsonObject>()["channel"];
      config.cellN = cellN;
      config.identity = identity;
      config.channel = channel;
      writeConfig();
      request->send(200, "text/plain", "Ok");
    } else {
      AsyncJsonResponse *response = new AsyncJsonResponse();
      JsonObject root = response->getRoot().to<JsonObject>();
      root["cellN"] = config.cellN;
      root["identity"] = config.identity;
      root["channel"] = config.channel;
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

TaskResult doServerWork(unsigned long now) {
  dnsServer.processNextRequest();
  ws.textAll(
    state.boardVoltage + String(",") + 
    state.encodedThrottleValue + String(",") +
    state.currentRSSI + String(",") +
    state.currentSNR
  );
  return { true, 0 };
}