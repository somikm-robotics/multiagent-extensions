#include "supervisory_agent_http_server.hpp"
#include <nlohmann/json.hpp>
#include "agent_utils.hpp"

SupervisoryAgentHttpServer::SupervisoryAgentHttpServer(const std::string &webRoot)
                                    :webRoot_(webRoot)
{}

void SupervisoryAgentHttpServer::setStatusMessageProvider(MessageProvider provider)
{
    statusMessageProvider_ = std::move(provider);
}

void SupervisoryAgentHttpServer::setHistoryMessageProvider(MessageProvider provider)
{
    historyMessageProvider_ = std::move(provider);
}

void SupervisoryAgentHttpServer::setDataProvider(DataProvider provider)
{
    locationsDataProvider_ = provider;
}

void SupervisoryAgentHttpServer::setLogger(Logger logger)
{
    logger_ = logger;
}

void SupervisoryAgentHttpServer::setOverrideMissionHandler(OverrideMissionServiceCaller handler)
{
    overrideMissionHandler_ = std::move(handler);
}

void SupervisoryAgentHttpServer::setScanIncidentHandler(ScanIncidentServiceCaller handler)
{
    scanIncidentHandler_ = std::move(handler);
}

void SupervisoryAgentHttpServer::start(const std::string &host, int port)
{
    httpServer_.new_task_queue = [] { return new httplib::ThreadPool(4);};
    registerRoutes();
    httpServer_.listen(host, port);
}

void SupervisoryAgentHttpServer::stop()
{
    httpServer_.stop();
}

void SupervisoryAgentHttpServer::registerRoutes()
{
    if(logger_)
        logger_("In register Routes");
    httpServer_.Get("/", [this](const httplib::Request&, httplib::Response& res) {
        serveFile(res, "/index.html", "text/html");
    });

    httpServer_.Get("/supervisory_agent_dashboard_ui.js", [this](const httplib::Request&, 
                httplib::Response& res) { 
                    serveFile(res, "/supervisory_agent_dashboard_ui.js", "application/javascript");
    });
    httpServer_.Get("/supervisory_agent_dashboard_styles.css", [this](const httplib::Request&, 
                    httplib::Response& res){
                        serveFile(res, "/supervisory_agent_dashboard_styles.css", "text/css");

    });

    httpServer_.Get("/mines/locations", [this](const httplib::Request&, 
                                                httplib::Response& res){
        // if(logger_)
        //     logger_("getting locations");                                                                                                
        auto locationsJSon = locationsDataProvider_();                                                    
        // if(logger_)
        // {
        //     logger_("locations: " + locationsJSon);    
        //    // logger_("locations size: " + std::to_string(locationsJSon.size()));
        // }
        res.body = locationsJSon;
        res.set_header("Content-Type", "application/json");
        res.set_header("Access-Control-Allow-Origin", "*");  
        res.set_header("Content-Length", std::to_string(locationsJSon.size()));                                    
        res.set_content(locationsJSon, "application/json");
        //res.set_content(std::string("[\"l1\",\"l2\"]"), "application/json");
    });

    // httpServer_.Get("/sample/test", [](const httplib::Request&, httplib::Response& res) {
    //     //res.set_content(std::string("{\"items\":[\"a\",\"b\"]}"), "application/json");
    //     //res.set_content(std::string("[\"a\",\"b\"]"), "application/json");
    //     res.set_content(std::string("[\"l1\",\"l2\"]"), "application/json");
    //     //res.set_content("{\"test\":\"ok\"}", "application/json");

    // });

    httpServer_.Get("/status/stream", [this](const httplib::Request&, httplib::Response& res) {
        // if(logger_)
        //     logger_("in status stream");                                                                                                
        res.set_header("Cache-Control", "no-cache");
        res.set_header("Connection",    "keep-alive");
        res.set_chunked_content_provider("text/event-stream",
            [this](size_t /*offset*/, httplib::DataSink& sink) -> bool {
                if (!statusMessageProvider_) return false;
                std::string event = "data: " + statusMessageProvider_() + "\n\n";
                return sink.write(event.c_str(), event.size());
            });
    });

    httpServer_.Get("/history/stream", [this](const httplib::Request&, httplib::Response& res) {
        // if(logger_)
        //     logger_("in history stream");                                                                                                
        res.set_header("Cache-Control", "no-cache");
        res.set_header("Connection",    "keep-alive");
        res.set_chunked_content_provider("text/event-stream",
            [this](size_t /*offset*/, httplib::DataSink& sink) -> bool {
                if (!historyMessageProvider_) return false;
                std::string event = "data: " + historyMessageProvider_() + "\n\n";
                return sink.write(event.c_str(), event.size());
            });
    });

    httpServer_.Post("/ui/override/mission", [this](const httplib::Request& req, 
                            httplib::Response& res){
        // if(logger_)
        //     logger_("in ui override mission - http server");                                    
        auto jObj = nlohmann::json::parse(req.body);
        auto [success, message] = overrideMissionHandler_ 
                                    ? overrideMissionHandler_(jObj["label"], jObj["action"])
                                    : std::make_pair(false, 
                                                        std::string("No override mission handler registered"));
        res.set_content(toJSon(success, message), "application/json");                                                        
    });

    httpServer_.Post("/scan/incident", [this](const httplib::Request& req, 
                            httplib::Response& res){
        // if(logger_)
        //     logger_("in scan incident - http server");                                                                 
        auto jObj = nlohmann::json::parse(req.body);      
        
        //logger_(std::string("is scanHandler set ? ") + (scanIncidentHandler_ ? "yes" : "no"));
        //logger_("location Name: " + std::string(jObj["locationName"]));                                
        auto [success, message] = scanIncidentHandler_ 
                                    ? scanIncidentHandler_(jObj["locationName"])
                                    : std::make_pair(false, 
                                                        std::string("No scan incident handler registered"));
        //logger_("In Scan Incident - https server - response message: " + message);                                                        
        res.set_content(toJSon(success, message), "application/json");
    });

}

void SupervisoryAgentHttpServer::serveFile(httplib::Response &res, 
    const std::string &path, const std::string &contentType)
{
    //std::cout << "path: " << webRoot_ + path;
    std::ifstream file(webRoot_ + path);
    if(!file) {
        res.status = 404;
        res.set_content("Not found - " + webRoot_ + path, "text/plain");
        return;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    res.set_content(buffer.str(), contentType);
}

std::string SupervisoryAgentHttpServer::toJSon(bool success, const std::string &message)
{
    nlohmann::json jObj;
    jObj["success"] = success;
    jObj["message"] = message;

    return jObj.dump();
}


