#ifndef TUNEFILTERDECIMATE_I_IMPL_H
#define TUNEFILTERDECIMATE_I_IMPL_H

#include "TuneFilterDecimate_base.h"

class TuneFilterDecimate_i : public TuneFilterDecimate_base
{
    ENABLE_LOGGING
    public:
        TuneFilterDecimate_i(const char *uuid, const char *label);
        ~TuneFilterDecimate_i();

        void constructor();

        int serviceFunction();
};

#endif // TUNEFILTERDECIMATE_I_IMPL_H
