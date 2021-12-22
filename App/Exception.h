#ifndef AEROTECH_EXCEPTION_H_HEADER_HAS_BEEN_INCLUDED
#define AEROTECH_EXCEPTION_H_HEADER_HAS_BEEN_INCLUDED

#include <exception>

#include <A3200.h>
#include <A3200Error.h>
#include <A3200ErrorCode.h>

namespace aerotech
{

class A3200Exception : public std::exception {

    uint32_t mCode;

    public:
        A3200Exception(const char* msg, uint32_t code) : std::exception(msg), mCode(code)
        {
        }

        uint32_t code() const { return mCode; }


};


}

#endif // AEROTECH_EXCEPTION_H_HEADER_HAS_BEEN_INCLUDED
