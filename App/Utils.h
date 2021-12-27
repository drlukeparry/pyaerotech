#ifndef AEROTECH_APP_UTILS_H_HEADER_HAS_BEEN_INCLUDED
#define AEROTECH_APP_UTILS_H_HEADER_HAS_BEEN_INCLUDED

#include "AEROTECH_Export.h"

#include <string>
#include <codecvt>
#include <locale>

namespace aerotech {

// Forward declaration
AEROTECH_EXPORT std::string UTF16toASCII(std::u16string utf16_string);
AEROTECH_EXPORT std::u16string ASCIItoUTF16(std::string ascii_string);

} //

#endif // AEROTECH_APP_UTILS_H_HEADER_HAS_BEEN_INCLUDED

