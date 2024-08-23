#include "log.h"
#include "error.h"
namespace collect_node_iot {

std::string getMessageByCode(int code)
{
    std::string out;

    switch (code)
    {
    case IOT_OK:
        out = "OK";
        break;
    case IOT_CONNECTION_LOST:
        out = "Iot Connection Lost";
        break;
    case IOT_REQUEST_TIMEOUT:
        out = "Request time out";
        break;
    case IOT_JSON_INVALID:
        out = "Json invalid";
        break;
    case IOT_SHADOW_NO_DESIRE:
        out = "Shadow doc has no desire data";
        break;
    default:
        out = Aws::Crt::ErrorDebugString(code);
        break;
    }

    return out;
}

}