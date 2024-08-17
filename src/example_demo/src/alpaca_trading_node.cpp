#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>

using json = nlohmann::json;
using namespace std::chrono_literals;

class AlpacaTradingNode : public rclcpp::Node {
public:
    AlpacaTradingNode() : Node("alpaca_trading_node") {
        this->declare_parameter("api_key", "");
        this->declare_parameter("api_secret", "");
        this->declare_parameter("base_url", "https://paper-api.alpaca.markets");  // Use paper trading by default

        api_key_ = this->get_parameter("api_key").as_string();
        api_secret_ = this->get_parameter("api_secret").as_string();
        base_url_ = this->get_parameter("base_url").as_string();

        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl_ = curl_easy_init();

        RCLCPP_INFO(this->get_logger(), "Alpaca Trading Node initialized");

        // Example: Get account information on startup
        get_account_info();
    }

    ~AlpacaTradingNode() {
        curl_easy_cleanup(curl_);
        curl_global_cleanup();
    }

private:
    void get_account_info() {
        std::string url = base_url_ + "/v2/account";
        std::string response = make_request(url, "GET");
        
        try {
            json account_info = json::parse(response);
            RCLCPP_INFO(this->get_logger(), "Account Status: %s", account_info["status"].get<std::string>().c_str());
            RCLCPP_INFO(this->get_logger(), "Buying Power: $%s", account_info["buying_power"].get<std::string>().c_str());
        } catch (json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
        }
    }

    void place_order(const std::string& symbol, int quantity, const std::string& side, const std::string& type, const std::string& time_in_force) {
        std::string url = base_url_ + "/v2/orders";
        
        json order_data = {
            {"symbol", symbol},
            {"qty", quantity},
            {"side", side},
            {"type", type},
            {"time_in_force", time_in_force}
        };

        std::string response = make_request(url, "POST", order_data.dump());

        try {
            json order_response = json::parse(response);
            RCLCPP_INFO(this->get_logger(), "Order placed: ID %s, Status %s", 
                        order_response["id"].get<std::string>().c_str(),
                        order_response["status"].get<std::string>().c_str());
        } catch (json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
        }
    }

    std::string make_request(const std::string& url, const std::string& method, const std::string& data = "") {
        if (!curl_) {
            RCLCPP_ERROR(this->get_logger(), "CURL not initialized");
            return "";
        }

        std::string readBuffer;
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, ("APCA-API-KEY-ID: " + api_key_).c_str());
        headers = curl_slist_append(headers, ("APCA-API-SECRET-KEY: " + api_secret_).c_str());
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &readBuffer);

        if (method == "POST") {
            curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, data.c_str());
        } else if (method != "GET") {
            curl_easy_setopt(curl_, CURLOPT_CUSTOMREQUEST, method.c_str());
        }

        CURLcode res = curl_easy_perform(curl_);
        if (res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
        }

        curl_slist_free_all(headers);
        return readBuffer;
    }

    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
        ((std::string*)userp)->append((char*)contents, size * nmemb);
        return size * nmemb;
    }

    CURL *curl_;
    std::string api_key_;
    std::string api_secret_;
    std::string base_url_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AlpacaTradingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}