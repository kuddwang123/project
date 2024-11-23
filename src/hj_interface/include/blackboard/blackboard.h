#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include <iostream>
#include <string>
#include <memory>
#include <stdint.h>
#include <unordered_map>

#include "blackboard/safe_any.hpp"


namespace BT
{
// This is the "backend" of the blackboard.
// To create a new blackboard, user must inherit from BlackboardImpl
// and override set and get.
typedef std::pair<SafeAny::Any, bool> blackboard_val;
class StaticBlackboardValue {
 public:
  StaticBlackboardValue(blackboard_val* in, const std::string& key) {
    if (nullptr == in) {  // not allow null
      std::string error_str = "in StaticBlackboardValue, the " + key + "\'s value = null";
      throw std::runtime_error(error_str);
    }
    b_ptr_ = in;
  };  // not allow to init later;
  StaticBlackboardValue(const StaticBlackboardValue& in) {
    b_ptr_ = in.b_ptr_;
  };
  StaticBlackboardValue& operator=(const StaticBlackboardValue& in) {
    if (this != &in) {
      b_ptr_ = in.b_ptr_;
    }
    return *this;
  };
  template <typename T>
  void set(T&& value) {
    b_ptr_->first = SafeAny::Any(std::forward<T>(value));
    b_ptr_->second = true;
  }
  template <typename T>
  bool get(T& value) const {
    if (false == b_ptr_->second) {
      return false;
    }
    value = b_ptr_->first.cast<T>();
    return true;
  }
  const SafeAny::Any* getAny() const {
    if (false == b_ptr_->second) {
      return nullptr;
    }
    return &(b_ptr_->first);
  }
  bool contains() const { return b_ptr_->second; }

  void erase() { b_ptr_->second = false; }
 private:
  blackboard_val* b_ptr_;
};
class BlackboardImpl
{
  public:
    virtual ~BlackboardImpl() = default;

    virtual const SafeAny::Any* get(const std::string& key) const = 0;
    virtual blackboard_val* getAgent(const std::string& key){std::cerr<<"check the use of getAgent with key"<<key<<std::endl;return nullptr;};
    virtual void set(const std::string& key, SafeAny::Any &&value) = 0;
    virtual bool contains(const std::string& key) const = 0;
    virtual void erase(const std::string& key) = 0;
};

// This is the "frontend" to be used by the developer.
//
// Even if the abstract class BlackboardImpl can be used directly,
// the templatized methods set() and get() are more user-friendly
class Blackboard
{
    // This is intentionally private. Use Blackboard::create instead
    Blackboard(std::unique_ptr<BlackboardImpl> base) : impl_(std::move(base))
    {
    }

  public:
    typedef std::shared_ptr<Blackboard> Ptr;

    Blackboard() = delete;

    /** Use this static method to create an instance of the BlackBoard
    *   to share among all your NodeTrees.
    */
    template <typename ImplClass, typename... Args>
    static Blackboard::Ptr create(Args... args)
    {
        std::unique_ptr<BlackboardImpl> base(new ImplClass(args...));
        return std::shared_ptr<Blackboard>(new Blackboard(std::move(base)));
    }

    virtual ~Blackboard() = default;

    /** Return true if the entry with the given key was found.
     *  Note that this method may throw an exception if the cast to T failed.
     *
     * return true if succesful
     */
    template <typename T>
    bool get(const std::string& key, T& value) const
    {
        if (!impl_)
        {
            return false;
        }
        const SafeAny::Any* val = impl_->get(key);
        if (!val)
        {
            return false;
        }

#if defined(__amd64__) || defined(__amd64) || defined(__x86_64__) || defined(__x86_64) || defined(_M_X64) || defined(_M_AMD64)
#define ENALBE_SET_ANY_DBG_STR
#endif
#ifdef ENALBE_SET_ANY_DBG_STR
        SafeAny::Any::DebugString() = key.c_str();
#endif
        try
        {
          value = val->cast<T>();
        }
        catch(const std::exception& e)
        {
          std::cerr << "key: " << key.c_str() << "--->" << e.what() << '\n';
          throw e;
        }
#ifdef ENALBE_SET_ANY_DBG_STR
        SafeAny::Any::DebugString() = "";
#endif
        return true;
    }

    const SafeAny::Any* getAny(const std::string& key) const
    {
        if (!impl_)
        {
            return nullptr;
        }
        return impl_->get(key);
    }

    StaticBlackboardValue getAgent(const std::string& key)
    {
        if (!impl_)
        {
            throw std::runtime_error("impl_ not exist");
        }
        return  StaticBlackboardValue(impl_->getAgent(key), key);
    }

    template <typename T>
    T get(const std::string& key) const
    {
        T value;
        bool found = get(key, value);
        if (!found)
        {
            throw std::runtime_error("Missing key");
        }
        return value;
    }

    // 为了防止用Any保存另一个Any造成的无限递归构造，需要重载这个函数
    void set(const std::string& key, SafeAny::Any& value)
    {
        if (impl_)
        {
            const SafeAny::Any* pvalue = &value;
            impl_->set(key, SafeAny::Any(*pvalue));
        }
    }
    /// Update the entry with the given key
    template <typename T>
    void set(const std::string& key, T&& value)
    {
        if (impl_)
        {
            impl_->set(key, SafeAny::Any(std::forward<T>(value)));
        }
    }

    bool contains(const std::string& key) const
    {
        return (impl_ && impl_->contains(key));
    }

    void erase(const std::string& key)
    {
        if (impl_)
        {
            impl_->erase(key);
        }
    }

  private:
    std::unique_ptr<BlackboardImpl> impl_;
};
}

#endif   // BLACKBOARD_H
