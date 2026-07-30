#pragma once

#include <httplib.h>

class SupervisoryAgentHttpServer 
{
public:
    using MessageProvider = std::function<std::string()>;
    using OverrideMissionServiceCaller = 
        std::function<std::pair<bool, std::string>(const std::string&, const std::string&)>;

    using ScanIncidentServiceCaller = 
            std::function<std::pair<bool, std::string>(const std::string&)>;
    using DataProvider = std::function<std::string()>;
    using Logger = std::function<void(const std::string&)>;

    SupervisoryAgentHttpServer(const std::string& webRoot);

    void setStatusMessageProvider(MessageProvider provider);
    void setHistoryMessageProvider(MessageProvider provider);
    void setDataProvider(DataProvider provider);
    void setLogger(Logger logger);

    void setOverrideMissionHandler(OverrideMissionServiceCaller handler);
    void setScanIncidentHandler(ScanIncidentServiceCaller handler);

    void start(const std::string& host, int port);
    void stop();

private:
    void registerRoutes();
    void serveFile(httplib::Response& res, const std::string& path, 
                    const std::string& contentType); 
    std::string toJSon(bool success, const std::string& message);

    httplib::Server httpServer_;
    std::string webRoot_;
    MessageProvider statusMessageProvider_;
    MessageProvider historyMessageProvider_;
    DataProvider locationsDataProvider_;
    OverrideMissionServiceCaller overrideMissionHandler_;
    ScanIncidentServiceCaller scanIncidentHandler_;    
    Logger logger_;                

};