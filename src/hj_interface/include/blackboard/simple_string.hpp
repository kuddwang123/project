#ifndef SIMPLE_STRING_HPP
#define SIMPLE_STRING_HPP

#include <string>
#include <cstring>

namespace SafeAny
{
// Version of string that uses only two words. Good for small object optimization in linb::any
class SimpleString
{
  public:
    SimpleString(const std::string& str) : SimpleString(str.data(), str.size())
    {
    }
    SimpleString(const char* data) : SimpleString(data, strlen(data))
    {
    }

    /// 借鉴fbstring的SSO实现，在原本的基础上实现SSO
    SimpleString(const char* data, std::size_t size)
    {
        if (size <= maxSmallSize)
        {
            initSmall(data, size);
        }
        else
        {
            initLarge(data, size);
        }
    }

    SimpleString(const SimpleString& other) : SimpleString(other.data(), other.size())
    {
    }

    // 原本没有禁用赋值函数，用默认的话会double free
    SimpleString& operator=(const SimpleString& other) = delete;

    ~SimpleString()
    {
        if (isLarge() && ml_._data)
        {
            delete[] ml_._data;
        }
    }

    std::string toStdString() const
    {
        if (isLarge())
        {
            return std::string(ml_._data, ml_.size());
        }
        else
        {
            return std::string(small_, smallSize());
        }
    }

    const char* data() const
    {
        if (isLarge())
        {
            return ml_._data;
        }
        else
        {
            return small_;
        }
    }

    std::size_t size() const
    {
        if (isLarge())
        {
            return ml_.size();
        }
        else
        {
            return smallSize();
        }
    }

    uint8_t isLarge() const
    {
        return bytes_[lastChar] & categoryExtractMask;
    }

  private:
    void initSmall(const char* const data, const size_t size)
    {
        if (size != 0)
        {
            memcpy(bytes_, data, size);
        }
        bytes_[size] = 0;
        setSmallSize(size);
    }
    void initLarge(const char* const data, const size_t size)
    {
        ml_._data = new char[size + 1];
        memcpy(ml_._data, data, size);
        ml_._data[size] = '\0';
        ml_.setSize(size);
    }
    void setSmallSize(size_t s)
    {
        small_[maxSmallSize] = char(maxSmallSize - s);
    }
    size_t smallSize() const
    {
        return size_t(maxSmallSize - small_[maxSmallSize]);
    }
    struct MediumLarge
    {
        char* _data;
        size_t _size;
        size_t size() const
        {
            return _size & sizeExtractMask;
        }
        void setSize(size_t cap)
        {
            _size = cap | (static_cast<size_t>(categoryExtractMask) << kCategoryShift);
        }
    };

    // 将短字符串存储在栈区，用union存储，可以有效节省内存
    union
    {
        uint8_t bytes_[sizeof(MediumLarge)];   // 存储所有字节
        char small_[sizeof(MediumLarge) / sizeof(char)];
        MediumLarge ml_;
    };
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    constexpr static bool kIsLittleEndian = true;
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#error "big endian is not supported"
#else
#error "unknown byte order"
#endif   // __BYTE_ORDER__
    constexpr static size_t lastChar = sizeof(MediumLarge) - 1;
    constexpr static size_t maxSmallSize =
        lastChar / sizeof(char);   // 64位平台为15个字节，32位平台为7个字节
    constexpr static uint8_t categoryExtractMask = 0x80;
    constexpr static size_t kCategoryShift = (sizeof(size_t) - 1) * 8;
    constexpr static size_t sizeExtractMask = ~(size_t(categoryExtractMask) << kCategoryShift);
};
}   // namespace SafeAny

#endif   // SIMPLE_STRING_HPP
