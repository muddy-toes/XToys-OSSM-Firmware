#include <sstream>
#include <WiFi.h>
#include <HTTPSServer.hpp>
#include <SSLCert.hpp>
#include <HTTPRequest.hpp>
#include <HTTPResponse.hpp>
#include <WebsocketHandler.hpp>

#include "config.h"

using namespace httpsserver;

namespace WebsocketManager {

    // Callback function to call any time a message is received
    extern void (*msgReceivedCallback)(String);

    // As websockets are more complex, they need a custom class that is derived from WebsocketHandler
    class MessageHandler : public WebsocketHandler {
        public:
            // This method is called by the webserver to instantiate a new handler for each
            // client that connects to the websocket endpoint
            static WebsocketHandler* create();

            // This method is called when a message arrives
            void onMessage(WebsocketInputStreambuf * input);

            // Handler function on connection close
            void onClose();
    };

    void handleRoot(HTTPRequest * req, HTTPResponse * res);
    void handle404(HTTPRequest * req, HTTPResponse * res);

    void setup(void (*msgReceivedCallback)(String));
    void loop();
    void sendMessage(String msg);

    extern SSLCert cert;
    extern HTTPSServer secureServer;
    extern MessageHandler* activeClients[MAX_CLIENTS];
}