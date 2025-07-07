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
      bool isDual = json.as<JsonObject>()["isDual"];
      config.cellN = cellN;
      config.identity = identity;
      config.channel = channel;
      config.isDual = isDual;
      writeConfig();
      request->send(200, "text/plain", "Ok");
    } else {
      AsyncJsonResponse *response = new AsyncJsonResponse();
      JsonObject root = response->getRoot().to<JsonObject>();
      root["cellN"] = config.cellN;
      root["identity"] = config.identity;
      root["channel"] = config.channel;
      root["isDual"] = config.isDual;
      response->setLength();
      request->send(response);
    }
  });
}

static AsyncCallbackJsonWebHandler *calibrationHandler = new AsyncCallbackJsonWebHandler("/calibration");
void configureCalibrationHandler() {
  calibrationHandler->setMethod(HTTP_POST | HTTP_GET);
  calibrationHandler->onRequest([](AsyncWebServerRequest *request, JsonVariant &json) {
    if(request->method() == HTTP_POST) {
      unsigned int calBrake = json.as<JsonObject>()["calBrake"];
      unsigned int calAcc = json.as<JsonObject>()["calAcc"];
      unsigned int centerBrake = json.as<JsonObject>()["centerBrake"];
      unsigned int centerAcc = json.as<JsonObject>()["centerAcc"];
      bool inverted = json.as<JsonObject>()["inverted"];
      config.calAcc = calAcc;
      config.calBrake = calBrake;
      config.centerAcc = centerAcc;
      config.centerBrake = centerBrake;
      config.inverted = inverted;
      writeCalibration();
      request->send(200, "text/plain", "Ok");
    } else {
      AsyncJsonResponse *response = new AsyncJsonResponse();
      JsonObject root = response->getRoot().to<JsonObject>();
      root["calBrake"] = config.calBrake;
      root["calAcc"] = config.calAcc;
      root["centerAcc"] = config.centerAcc;
      root["centerBrake"] = config.centerBrake;
      root["inverted"] = config.inverted;
      response->setLength();
      request->send(response);
    }
  });
}

static AsyncCallbackJsonWebHandler *rxSetupHandler = new AsyncCallbackJsonWebHandler("/rxsetup");
void configureRXSetupHandler() {
  rxSetupHandler->setMethod(HTTP_POST | HTTP_GET);
  rxSetupHandler->onRequest([](AsyncWebServerRequest *request, JsonVariant &json) {
    if(request->method() == HTTP_POST) {
      state.forceRxSetup = true;
      request->send(200, "text/plain", "Ok");
    } else {
      request->send(404, "text/plain", "Not Found");
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
  WiFi.softAP("Unfancy Remote TX");
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.setTTL(300);
  dnsServer.start(53, "*", WiFi.softAPIP());

  configureSettingsHandler();
  configureCalibrationHandler();
  configureRXSetupHandler();

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.addHandler(new CaptivePortalHandler()).setFilter(ON_AP_FILTER);
  server.addHandler(settingsHandler);
  server.addHandler(calibrationHandler);
  server.addHandler(rxSetupHandler);
  server.onNotFound([&](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
  });
  
  server.begin();
}

TaskResult doServerWork(unsigned long now) {
  dnsServer.processNextRequest();
  ws.textAll(
    state.batteryVoltage + String(",") + 
    state.boardVoltage + String(",") + 
    state.rawThrottle1Value + String(",") + 
    state.rawThrottle2Value + String(",") +
    state.encodedThrottleValue
  );
  return { true };
}