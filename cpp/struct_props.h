#ifndef STRUCTPROPS_H
#define STRUCTPROPS_H

/*******************************************************************************************

    AUTO-GENERATED CODE. DO NOT MODIFY

*******************************************************************************************/

#include <ossie/CorbaUtils.h>
#include <CF/cf.h>
#include <ossie/PropertyMap.h>

struct filterProps_struct {
    filterProps_struct ()
    {
        FFT_size = 128;
        TransitionWidth = 800;
        Ripple = 0.01;
    };

    static std::string getId() {
        return std::string("filterProps");
    };

    CORBA::ULong FFT_size;
    double TransitionWidth;
    double Ripple;
};

inline bool operator>>= (const CORBA::Any& a, filterProps_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("FFT_size")) {
        if (!(props["FFT_size"] >>= s.FFT_size)) return false;
    }
    if (props.contains("TransitionWidth")) {
        if (!(props["TransitionWidth"] >>= s.TransitionWidth)) return false;
    }
    if (props.contains("Ripple")) {
        if (!(props["Ripple"] >>= s.Ripple)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const filterProps_struct& s) {
    redhawk::PropertyMap props;
 
    props["FFT_size"] = s.FFT_size;
 
    props["TransitionWidth"] = s.TransitionWidth;
 
    props["Ripple"] = s.Ripple;
    a <<= props;
}

inline bool operator== (const filterProps_struct& s1, const filterProps_struct& s2) {
    if (s1.FFT_size!=s2.FFT_size)
        return false;
    if (s1.TransitionWidth!=s2.TransitionWidth)
        return false;
    if (s1.Ripple!=s2.Ripple)
        return false;
    return true;
}

inline bool operator!= (const filterProps_struct& s1, const filterProps_struct& s2) {
    return !(s1==s2);
}

#endif // STRUCTPROPS_H
