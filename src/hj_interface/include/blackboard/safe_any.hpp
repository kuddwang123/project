#ifndef SAFE_ANY_VARNUMBER_H
#define SAFE_ANY_VARNUMBER_H

#include <exception>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <string>
#include <cstring>
#include <type_traits>
#include <unordered_map>
#include <typeindex>
#include "any.hpp"
#include "demangle_util.h"
#include "convert_impl.hpp"

// SFINAE检测类型T是否重载了convert_any(const std::type_info, const void*, T&)
template <typename T>
struct has_ConvFromAny
{
  private:
    typedef char one;
    typedef struct
    {
        char data[2];
    } two;
    // 存在的话返回类型为 one
    template <typename U>
    static one test(decltype(convert_any(typeid(void), (void *)0, *((U*)0)))*);
    // 不存在的话返回类型为 two
    template <typename U>
    static two test(...);

  public:
    enum
    {
        value = sizeof(test<T>(0)) == sizeof(one)
    };
};
template <class T, typename std::enable_if<has_ConvFromAny<T>::value, int>::type = 0>
inline int ConvertFromAny(const std::type_info& type, const void* in, T& out)
{
    return convert_any(type, in, out);
}
template <class T, typename std::enable_if<!has_ConvFromAny<T>::value, int>::type = 0>
inline int ConvertFromAny(const std::type_info&, const void*, T&)
{
    return 1;
}

inline void convert_str_error(std::pair<const char*, size_t> in, const char* type_name)
{
    throw std::runtime_error("convert_str error. str = " + std::string(in.first, in.second) +
                             ". type_name = " + type_name);
}
inline void convert_str(std::pair<const char*, size_t> in, bool& dst)
{
    if (in.second == 5 && *(int*)in.first == *(int*)"fals" && in.first[4] == 'e') {
        dst = false;
    } else if (in.second == 4 && *(int*)in.first == *(int*)"true") {
        dst = true;
    } else {
        convert_str_error(in, "bool");
    }
}
inline void convert_str(std::pair<const char*, size_t> in, unsigned char& dst)
{
    if (in.second > 0 && in.second <= 3) {
        int ret = 0;
        for (int i = 0; i < int(in.second); ++i) {
            auto &c = in.first[i];
            if (c >= '0' && c <= '9') {
                ret = ret * 10 + (c - '0');
            } else {
                convert_str_error(in, "unsigned char 1");
            }
        }
        if (ret > 255) {
            convert_str_error(in, "unsigned char 3");
        }
        dst = ret;
    } else {
        convert_str_error(in, "unsigned char 2");
    }
}
// SFINAE检测类型T是否重载了convert_str(std::pair<const char *, size_t> in, T&)
template <typename T>
struct has_ConvFromStr2
{
  private:
    typedef char one;
    typedef struct
    {
        char data[2];
    } two;
    // 存在的话返回类型为 one
    template <typename U>
    static one test(decltype(convert_str(std::make_pair((const char*)0, size_t(0)), *((U*)0)))*);
    // 不存在的话返回类型为 two
    template <typename U>
    static two test(...);

  public:
    enum
    {
        value = sizeof(test<T>(0)) == sizeof(one)
    };
};
template <class T, typename std::enable_if<has_ConvFromStr2<T>::value, int>::type = 0>
inline int ConvertFromStr(std::pair<const char *, size_t> in, T& out)
{
    convert_str(in, out);
    return 0;
}
template <class T, typename std::enable_if<!has_ConvFromStr2<T>::value, int>::type = 0>
inline int ConvertFromStr(std::pair<const char *, size_t>, T&)
{
    return 1;
}
namespace SafeAny
{
// Rational: since type erased numbers will always use at least 8 bytes
// it is faster to cast everything to either double, uint64_t or int64_t.
class Any
{
    template <typename T>
    using EnableIntegral =
        typename std::enable_if<std::is_integral<T>::value || std::is_enum<T>::value>::type*;

    template <typename T>
    using EnableNonIntegral =
        typename std::enable_if<!std::is_integral<T>::value && !std::is_enum<T>::value>::type*;

    template <typename T>
    using EnableString = typename std::enable_if<std::is_same<T, std::string>::value>::type*;

    template <typename T>
    using EnableConstString = typename std::enable_if<std::is_same<T, const char *>::value>::type*;

    template <typename T>
    using EnableArithmetic = typename std::enable_if<std::is_arithmetic<T>::value>::type*;

    template <typename T>
    using EnableEnum = typename std::enable_if<std::is_enum<T>::value>::type*;

    template <typename T>
    using EnableUnknownType =
        typename std::enable_if<!std::is_arithmetic<T>::value && !std::is_enum<T>::value &&
                                !std::is_same<T, std::string>::value &&
                                !std::is_same<T, const char *>::value>::type*;

    template <typename T>
    using EnableHasConv =
        typename std::enable_if<has_ConvFromStr2<T>::value || has_ConvFromAny<T>::value, int>::type;
    template <typename T>
    using EnableHasNoConv =
        typename std::enable_if<!(has_ConvFromStr2<T>::value || has_ConvFromAny<T>::value),
                                int>::type;

  public:
    Any()
    {
    }

    ~Any() = default;

    Any(const double& value) : _any(value)
    {
    }

    Any(const uint64_t& value) : _any(value)
    {
    }

    Any(const float& value) : _any(double(value))
    {
    }

    Any(const std::string& str) : _any(SimpleString(str))
    {
    }

    Any(const char* str) : _any(SimpleString(str))
    {
    }

    Any(const Any &rhs) : _any(rhs._any)
    {
    }

    // all the other integrals are casted to int64_t
    template <typename T>
    explicit Any(const T& value, EnableIntegral<T> = 0) : _any(int64_t(value))
    {
    }

    // default for other custom types
    template <typename T>
    explicit Any(const T& value, EnableNonIntegral<T> = 0) : _any(value)
    {
    }

    // 这里还缺一种构造函数，就是输入其他对象时的的移动构造
    // 比如输入是一个std::vector的右值引用，就可以在构造Any时把输入值move进去，避免拷贝
    // 经过试验是可以实现的，但是会造成float类型的左值输入时不会调用对应的特殊重载
    // 因此，放弃实现对其他对象的移动构造，毕竟使用黑板的set时几乎没人加上std::move
    Any(Any &&rhs) :_any(std::move(rhs._any))
    {
    }
    Any& operator=(Any &&rhs) {
        _any = std::move(rhs._any);
        return *this;
    }

    Any& operator=(const Any &rhs) {
        _any = rhs._any;
        return *this;
    }

    // this is different from any_cast, because if allows safe
    // conversions between arithmetic values.
    template <typename T, EnableHasNoConv<T> = 0>
    T cast() const
    {
        const auto& type = _any.type();
        if (type == typeid(T))
        {
            return linb::any_cast<T>(_any);
        }
        else
        {
            return convert<T>();
        }
    }
    template <typename T, EnableHasConv<T> = 0>
    T cast() const
    {
        const auto& type = _any.type();
        if (type == typeid(T))
        {
            return linb::any_cast<T>(_any);
        }
        else
        {
            T ret{};
            if (type == typeid(SimpleString)) {
                // 存储的是个字符串，看看是否存在字符串转类型的函数
                const SimpleString* str = linb::any_cast<SimpleString>(&_any);
                if (ConvertFromStr<T>(std::make_pair(str->data(), str->size()), ret) == 0) {
                    return ret;
                }
            }
            // 直接调用特殊转换函数，靠ADL来查找，找不到特殊的就用默认的，默认的会返回1
            {
                if (ConvertFromAny<T>(_any.type(), _any.data(), ret) == 0)
                {
                    return ret;
                }
            }
            return convert<T>();
        }
    }

    const std::type_info& type() const noexcept
    {
        return _any.type();
    }
    static const char *&DebugString() {
        static const char *ins = "";
        return ins;
    }

  private:
    linb::any _any;

    //----------------------------

    template <typename DST>
    DST convert(EnableConstString<DST> = 0) const
    {
        const auto& type = _any.type();

        if (type == typeid(SimpleString))
        {
            return linb::any_cast<SimpleString>(&_any)->data();
        }

        throw errorMsg(typeid(DST).name());
    }

    template <typename DST>
    DST convert(EnableString<DST> = 0) const
    {
        const auto& type = _any.type();

        if (type == typeid(SimpleString))
        {
            return linb::any_cast<SimpleString>(_any).toStdString();
        }
        else if (type == typeid(int64_t))
        {
            return std::to_string(linb::any_cast<int64_t>(_any));
        }
        else if (type == typeid(uint64_t))
        {
            return std::to_string(linb::any_cast<uint64_t>(_any));
        }
        else if (type == typeid(double))
        {
            return std::to_string(linb::any_cast<double>(_any));
        }

        throw errorMsg(typeid(DST).name());
    }

    template <typename DST>
    DST convert(EnableArithmetic<DST> = 0) const
    {
        using details::convertNumber;
        DST out;

        const auto& type = _any.type();

        if (type == typeid(int64_t))
        {
            convertNumber<int64_t, DST>(linb::any_cast<int64_t>(_any), out);
        }
        else if (type == typeid(uint64_t))
        {
            convertNumber<uint64_t, DST>(linb::any_cast<uint64_t>(_any), out);
        }
        else if (type == typeid(double))
        {
            convertNumber<double, DST>(linb::any_cast<double>(_any), out);
        }
        else
        {
            throw errorMsg(typeid(DST).name());
        }
        return out;
    }

    template <typename DST>
    DST convert(EnableEnum<DST> = 0) const
    {
        using details::convertNumber;

        const auto& type = _any.type();

        if (type == typeid(int64_t))
        {
            uint64_t out = linb::any_cast<int64_t>(_any);
            return static_cast<DST>(out);
        }
        else if (type == typeid(uint64_t))
        {
            uint64_t out = linb::any_cast<uint64_t>(_any);
            return static_cast<DST>(out);
        }

        throw errorMsg(typeid(DST).name());
    }

    template <typename DST>
    DST convert(EnableUnknownType<DST> = 0) const
    {
        throw errorMsg(typeid(DST).name());
    }

    std::runtime_error errorMsg(const char *type_name) const;
};

}   // end namespace VarNumber

#endif   // VARNUMBER_H
