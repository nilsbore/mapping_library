#include "conversions.h"

namespace convenience {

std::string rgb_to_hex_string(unsigned r, unsigned g, unsigned b)
{
    char hexcol[16];
    snprintf(hexcol, sizeof(hexcol), "#%02x%02x%02x", r, g, b);
    std::string rtn(hexcol);
    return rtn;
}

}
