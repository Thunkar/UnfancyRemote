#include "wifiSetup.h"

const char* accessPointName = "Unfance Remote AP";

DNSServer dnsServer;
AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Captive Portal</title>
</head>
<body>
  <h1>Welcome to the Captive Portal</h1>
  <p>Please agree to the terms to access the internet.</p>
  <form>
    <input type="checkbox" id="terms" name="terms">
    <label for="terms">I agree to the terms of service</label><br>
    <input type="submit" value="Continue">
  </form>
</body>
</html>
)rawliteral";


class CaptivePortalHandler : public AsyncWebHandler {
public:
  CaptivePortalHandler() {}
  virtual ~CaptivePortalHandler() {}

  bool canHandle(AsyncWebServerRequest *request){
    return request->url() == "/";
  }

  void handleRequest(AsyncWebServerRequest *request) {
    if (request->method() == HTTP_GET && request->url() == "/") {
      request->send(200, "text/html", index_html);
    } else {
      request->send(200, "text/html", index_html);
    }
  }
};

void setupServer(){
  server.addHandler(new CaptivePortalHandler()).setFilter(ON_AP_FILTER);

  server.onNotFound([&](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html); 
  });
}