#include "Utils.h"
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <iomanip>
#include <openssl/evp.h>
#include <openssl/aes.h>
#include <openssl/err.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "log.h"

namespace collect_node_iot {

namespace utils {
bool hexStringToUnsignedCharArray(const std::string& hexstring, unsigned char* byteArray, size_t len)
{
    if (hexstring.size()/2 != len)
        return false;
    for (size_t i = 0; i < len; ++i) {
        std::string byteString = hexstring.substr(i * 2, 2);
        byteArray[i] = static_cast<unsigned char>(std::stoi(byteString, nullptr, 16));
    }
    return true;
}

bool asciiStringToUnsignedCharArray(const std::string& asciistring, unsigned char* byteArray, size_t len)
{
    if (asciistring.size() != len)
        return false;
    for (size_t i = 0; i < len; ++i) {
        byteArray[i] = asciistring[i];
    }
    return true;
}

std::string aesCbc128Encode(const std::string& plaintext, unsigned char* key, unsigned char* iv)
{
    std::string aesenc;
    int plaintext_len = plaintext.size();

    int paddingLength = AES_BLOCK_SIZE - (plaintext_len % AES_BLOCK_SIZE);
    int ciperlen = plaintext_len + paddingLength;
    unsigned char* ciphertext = new unsigned char[ciperlen];
    memset(ciphertext, 0, ciperlen);
    //unsigned char ciphertext[plaintext_len + paddingLength] = {0};
    //int ciperlen = sizeof(ciphertext)/sizeof(char);

    AES_KEY aesKey;
    if (AES_set_encrypt_key(key, 128, &aesKey) != 0) {
        HJ_ERROR("aes encrypt key error\n");
        ERR_print_errors_fp(stderr);
    } else {
        AES_cbc_encrypt(reinterpret_cast<const unsigned char*>(plaintext.data()), ciphertext, plaintext_len, &aesKey, iv, AES_ENCRYPT);
        aesenc.assign(reinterpret_cast<char*>(ciphertext), ciperlen);
    }

    delete[] ciphertext;
    return aesenc;
}

std::string aesCbc128Decode(const unsigned char* ciphertext, ssize_t ciphertext_len, unsigned char* key, unsigned char* iv)
{    
    EVP_CIPHER_CTX *ctx;
    int len;

    //unsigned char plaintext[ciphertext_len] = {0};
    
    // Create and initialize the context
    if (!(ctx = EVP_CIPHER_CTX_new())) {
        HJ_ERROR("evp new fail\n");
        return "";
    }

    // Initialize decryption
    if (1 != EVP_DecryptInit_ex(ctx, EVP_aes_128_cbc(), NULL, key, iv)) {
        EVP_CIPHER_CTX_free(ctx);
        HJ_ERROR("Decryption init failed\n");
        return "";
    }

    EVP_CIPHER_CTX_set_padding(ctx, 0);

    unsigned char* plaintext = new unsigned char[ciphertext_len]; 
    memset(plaintext, 0, ciphertext_len);

    if (1 != EVP_DecryptUpdate(ctx, plaintext, &len, ciphertext, ciphertext_len)) {
        EVP_CIPHER_CTX_free(ctx);
        delete[] plaintext;
	    HJ_ERROR("Decryption process failed\n");
        return "";
    }

    // Decrypt the final block (which may contain padding)
    if (1 != EVP_DecryptFinal_ex(ctx, plaintext + len, &len)) {
        EVP_CIPHER_CTX_free(ctx);
        delete[] plaintext;
	    HJ_ERROR("Decryption final failed\n");
        return "";
    }

    EVP_CIPHER_CTX_free(ctx);

    std::string out;
    out.assign(reinterpret_cast<char*>(plaintext), strlen(reinterpret_cast<const char*>(plaintext)));

    delete[] plaintext;

    return out;
}

std::string documentToString(const rapidjson::Document& doc) 
{
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);
    return buffer.GetString();
}

std::string documentToString(const rapidjson::Value& value) 
{
    rapidjson::Document doc;
    doc.CopyFrom(value, doc.GetAllocator());
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);
    return buffer.GetString();
}

std::string getIpAddrString()
{
    std::string ip_default = "127.0.0.1";

    struct ifaddrs *addrs, *tmp;
    getifaddrs(&addrs);
    tmp = addrs;

    while (tmp) {
        if (tmp->ifa_addr && tmp->ifa_addr->sa_family == AF_INET) {
            struct sockaddr_in *pAddr = (struct sockaddr_in *)tmp->ifa_addr;
            if (!strcmp(tmp->ifa_name, "lo"))  {
                tmp = tmp->ifa_next;
                continue;
            }
            ip_default = inet_ntoa(pAddr->sin_addr);
            break;
        }
        tmp = tmp->ifa_next;
    }

    freeifaddrs(addrs);

    HJ_INFO("get ip:%s",ip_default.c_str());
    return ip_default;
}

std::string xor_encrypt(const std::string& in)
{
    unsigned char key[] = {0x12, 0x34, 0x56, 0x78};
    std::string out = in;
    int key_index = 0;

    for (char& c : out) {
        c ^= key[key_index];
        key_index = (key_index + 1) % 4;
    }

    return out;
}

void replaceSpaceWithUnderline(std::string& str)
{
    for (int i = 0; i < str.size(); ++i) {
        if (str[i] == ' ') {
            str[i] = '_';
        }
    }
}

DevInfo::DevInfo(const char* filepath):
    ifs_(filepath)
{
    parse();
}

DevInfo::~DevInfo()
{
    if (this->openSucc())
        ifs_.close();
}

bool DevInfo::openSucc()
{
    return ifs_.is_open();
}

void DevInfo::parse()
{
    if (!ifs_.is_open())
        return;
    
    rapidjson::Document document;
    std::string jsonString((std::istreambuf_iterator<char>(ifs_)),
                    std::istreambuf_iterator<char>());

    if (!document.Parse(jsonString.data()).HasParseError()) {
        model_ = document["model"].GetString();
        sn_ = document["sn"].GetString();
        version_ = document["version"].GetString();
        bleName_ = document["bleName"].GetString();
        waddr_ = document["waddr"].GetString();
    }  else {
        ifs_.close();
    }
}

AwsCertParser::AwsCertParser(const rapidjson::Value& doc)
{
    parseDocument(doc);
}

AwsCertParser::AwsCertParser(const char* filepath)
{
    std::ifstream ifs(filepath);
    rapidjson::Document document;

    if (ifs.is_open()) {
        std::string jsonString((std::istreambuf_iterator<char>(ifs)),
                    std::istreambuf_iterator<char>());
        ifs.close();

        if (!document.Parse(jsonString.data()).HasParseError()) {
            parseDocument(document);
        }
    }
}

bool AwsCertParser::parseOk()
{
    return !cert_.empty() &&
           !prikey_.empty() &&
           !thingname_.empty() &&
           !endpoint_.empty();
}

void AwsCertParser::parseDocument(const rapidjson::Value& doc)
{
    if (doc.IsObject()) {
        if (doc.HasMember("awsCertCrt") && doc["awsCertCrt"].IsString()) {
            cert_ = doc["awsCertCrt"].GetString();
            HJ_INFO("get awsCertCrt");
        }
        if (doc.HasMember("awsCertPrivate") && doc["awsCertPrivate"].IsString()) {
            prikey_ = doc["awsCertPrivate"].GetString();
            HJ_INFO("get awsCertPrivate");
        }
        if (doc.HasMember("awsIotEndpoint") && doc["awsIotEndpoint"].IsString()) {
            endpoint_ = doc["awsIotEndpoint"].GetString();
            HJ_INFO("get awsIotEndpoint\n");
        }
        if (doc.HasMember("thingName") && doc["thingName"].IsString()) {
            thingname_ = doc["thingName"].GetString();
            HJ_INFO("get thingName");
        }
    } else {
        HJ_ERROR("doc not object\n");
    }
}

Curl::Curl(void)
	: curl_(NULL)
	,curlCode_(CURLE_OK)
	,flag_(kNone)
	,curlList_(NULL)
{
	curl_ = curl_easy_init();
	curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, writeCallback);
	curl_easy_setopt(curl_, CURLOPT_WRITEDATA, this);
    curl_easy_setopt(curl_, CURLOPT_VERBOSE, 1L);
}
 
 
Curl::~Curl(void)
{
	clearHeaderList();
	curl_easy_cleanup(curl_);
}
 
bool Curl::setUrl(const std::string& url)
{
    if ( url == url_ ) 
        return true;
    url_ = url;
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_URL, url_.data());
    return CURLE_OK == curlCode_;
}
 
bool Curl::setTimeout(int nSecond)
{
    if (nSecond < 0)
        return false;
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_TIMEOUT, nSecond);
    return CURLE_OK == curlCode_;
}
 
bool Curl::setConnectTimeout(int nSecond)
{
    if (nSecond < 0)
        return false;
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_CONNECTTIMEOUT, nSecond);
    return CURLE_OK == curlCode_;
}
 
bool Curl::addHeader(std::string key, std::string value)
{
    assert(key.size() > 0 && value.size() > 0);
    std::string strHeader(key);
    strHeader.append(": ");
    strHeader.append(value);
    curlList_ = curl_slist_append(curlList_, strHeader.c_str());
    curlCode_ = curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, curlList_);
    return CURLE_OK == curlCode_;
}
 
void Curl::clearHeaderList()
{
    if (curlList_)
    {
        curl_slist_free_all(curlList_);
        curlList_ = NULL;
    }
}
 
std::string Curl::getError() const
{
    return std::string(curl_easy_strerror(curlCode_));
}
 
bool Curl::post(std::string data)
{
    if (url_.empty())
        return false;
    curl_easy_setopt(curl_, CURLOPT_POST, 1);
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, data.c_str());
    curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, data.size());
    //curl_easy_setopt(curl_, CURLOPT_URL, url_.c_str());
    curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYHOST, 0L);
    flag_ = kPost;
    response_.clear();
    curlCode_ = curl_easy_perform(curl_);
    HJ_INFO("curl post ret code:%d\n", curlCode_);
    return CURLE_OK == curlCode_;
}
 
bool Curl::get(std::string url)
{
    assert(!url.empty());
	curl_easy_setopt(curl_, CURLOPT_HTTPGET, 1);
	curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
	curl_easy_setopt(curl_, CURLOPT_FOLLOWLOCATION, 1);//支持重定向
	curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYPEER, 0L);
	curl_easy_setopt(curl_, CURLOPT_SSL_VERIFYHOST, 0L);
	flag_ = kGet;
	response_.clear();
	curlCode_ = curl_easy_perform(curl_);
	return CURLE_OK == curlCode_;
}
 
const std::string& Curl::getResponse() const
{
	return response_;
}
 
const char* Curl::getResponseStr() const
{
	return response_.c_str();
}
 
size_t Curl::writeCallback(void* pBuffer, size_t nSize, size_t nMemByte, void* pParam)
{
	//把下载到的数据以追加的方式写入文件(一定要有a，否则前面写入的内容就会被覆盖了)
	Curl* pThis = (Curl*)pParam;
	int written = 0;
	switch(pThis->flag_)
	{
	case kDownload:
		{
            /*
			if ( pThis->m_hFile == INVALID_HANDLE_VALUE )
				return 0;
			if ( !WriteFile(pThis->m_hFile, pBuffer, nSize*nMemByte, &written, NULL) )
				return 0;
            */
		}
		break;
	case kPost:
	case kGet:
		{
			pThis->response_.append((const char*)pBuffer, nSize*nMemByte);
			written = nSize*nMemByte;
		}
		break;
	case kNone:
		break;
	}
	return written;
}
 
}

}