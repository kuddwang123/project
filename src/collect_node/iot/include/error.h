#pragma once
#include <string>
#include <aws/crt/Api.h>

namespace collect_node_iot {
enum {
    IOT_OK = 0,
    IOT_CONNECTION_LOST = 100,
    IOT_REQUEST_TIMEOUT = 101,
    IOT_JSON_INVALID = 102,
    IOT_SHADOW_NO_DESIRE = 103
};

std::string getMessageByCode(int code);

}