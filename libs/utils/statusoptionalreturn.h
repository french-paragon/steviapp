#ifndef STATUSOPTIONALRETURN_H
#define STATUSOPTIONALRETURN_H

#include <string>
#include <optional>

namespace StereoVisionApp {

template<typename R_T>
class StatusOptionalReturn {

public:
    static StatusOptionalReturn<R_T> error(const char* msg) {
        StatusOptionalReturn<R_T> ret;
        ret._val = std::nullopt;
        ret._error_msg = std::string(msg);

        return ret;
    }

    StatusOptionalReturn(R_T const& val) :
        _val(val)
    {

    }

    StatusOptionalReturn(R_T && val) :
        _val(std::move(val))
    {

    }

    inline bool isValid() const {
        return _val.has_value();
    }

    inline R_T& value() {
        return _val.value();
    }

    inline const char* errorMessage() const {
        return _error_msg.c_str();
    }

protected:

    StatusOptionalReturn() {
        _val = std::nullopt;
    }

    std::optional<R_T> _val;

    std::string _error_msg;

};

template<>
class StatusOptionalReturn<void> {

public:
    static StatusOptionalReturn<void> error(const char* msg) {
        StatusOptionalReturn<void> ret;
        ret._valid = false;
        ret._error_msg = std::string(msg);

        return ret;
    }



    StatusOptionalReturn() :
        _valid(true)
    {
    }

    inline bool isValid() const {
        return _valid;
    }

    inline const char* error() const {
        return _error_msg.c_str();
    }

protected:

    bool _valid;

    std::string _error_msg;

};

}

#endif // STATUSOPTIONALRETURN_H
