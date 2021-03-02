#pragma once
#ifndef _PHOXI_FEATURE_HPP
#define _PHOXI_FEATURE_HPP
#include "PhoXiCompilerDefines.h"

#include "PhoXiFeature.h"

#include <type_traits>

#ifdef DISABLE_LOCAL_INLINE
#   define LOCAL_INLINE
#else
#   define LOCAL_INLINE inline
#endif
namespace pho {
namespace api {
template<class T>
LOCAL_INLINE PhoXiFeatureSetGet<T>::PhoXiFeatureSetGet(const T &Other, PhoXiFeature<T> *Owner) :T(Other) {
    this->PhoXiFeatureSetGetOwner = Owner;
    Original = Other;
}
template<class T>
LOCAL_INLINE PhoXiFeatureSetGet<T>::~PhoXiFeatureSetGet() {
    if (PhoXiFeatureSetGetOwner->CanSet()) {
        if ((T) *this == Original) {
            //Do nothing
        } else {
            PhoXiFeatureSetGetOwner->SetValue(*this);
        }
    }
}

template<class T>
LOCAL_INLINE void PhoXiFeature<T>::BindGetFunction(std::function<T(PhoXiFeature<T> &)> Get) {
    this->Get = Get;
}
template<class T>
LOCAL_INLINE void PhoXiFeature<T>::BindGetFunction(GetFunction Get, PhoXiInterface *Owner) {
    this->Get = std::bind(Get, Owner, std::placeholders::_1);
}
template<class T>
LOCAL_INLINE void PhoXiFeature<T>::BindSetFunction(std::function<bool(const T &, PhoXiFeature<T> &)> Set) {
    this->Set = Set;
}
template<class T>
LOCAL_INLINE void PhoXiFeature<T>::BindSetFunction(SetFunction Set, PhoXiInterface *Owner) {
    this->Set = std::bind(Set, Owner, std::placeholders::_1, std::placeholders::_2);
}
template<class T>
LOCAL_INLINE void PhoXiFeature<T>::BindFunctions(std::function<T(PhoXiFeature<T> &)> Get,
                                                 std::function<bool(const T &, PhoXiFeature<T> &)> Set) {
    this->Get = Get;
    this->Set = Set;
}
template<class T>
LOCAL_INLINE void PhoXiFeature<T>::BindFunctions(GetFunction Get, SetFunction Set, PhoXiInterface *Owner) {
    if (Get) this->Get = std::bind(Get, Owner, std::placeholders::_1);
    if (Set) this->Set = std::bind(Set, Owner, std::placeholders::_1, std::placeholders::_2);
}
template<class T>
LOCAL_INLINE PhoXiFeature<T>::PhoXiFeature(const PhoXiFeature<T> &Other) :PhoXiFeatureInterface() {
    Error = Other.Error;
    Enabled = Other.Enabled;
    Value = Other.Value;
    Get = nullptr;
    Set = nullptr;
    Initiated = Other.Initiated;
    TypeName = typeid(T).name();
}
template<class T>
LOCAL_INLINE PhoXiFeature<T>::PhoXiFeature() :PhoXiFeatureInterface() {
    Value = T();
    Get = nullptr;
    Set = nullptr;
    TypeName = typeid(T).name();
}
template<class T>
LOCAL_INLINE void PhoXiFeature<T>::SetStoredValue(const T &Value) {
    this->Value = Value;
    Initiated = true;
}
template<class T>
LOCAL_INLINE bool PhoXiFeature<T>::CanGet() const {
    if (isEnabled() && Get && Gettable) {
        return true;
    } else {
        return false;
    }
}
template<class T>
LOCAL_INLINE bool PhoXiFeature<T>::CanSet() const {
    if (isEnabled() && Set && Settable) {
        return true;
    } else {
        return false;
    }
}
template<class T>
LOCAL_INLINE inline T PhoXiFeature<T>::GetValue() {
    if (isImplemented()) {
        if (isEnabled()) {
            if (CanGet()) {
                Error.clear();
                if (DefaultControl && !isOwnerConnected()) {
                    return GetStoredValue();
                } else {
                    T Result = Get(*this);
                    if (isLastOperationSuccessful()) {
                        return Result;
                    } else {
                        ErrorMessage("Error in " + GetOwnersName() + "." + Name + ".GetValue(): " + Error);
                        return T();
                    }
                }
            } else {
                Error = "Feature can not be read";
            }
        } else {
            Error = "Feature is not Enabled";
        }
    } else {
        Error = "Feature is not Implemented";
    }
    ErrorMessage("Error in " + GetOwnersName() + "." + Name + ".GetValue(): " + Error);
    return T();
}
template<class T>
LOCAL_INLINE inline bool PhoXiFeature<T>::SetValue(const T &Value) {
    if (isImplemented()) {
        if (isEnabled()) {
            if (CanSet()) {
                Error.clear();
                if (DefaultControl && !isOwnerConnected()) {
                    SetStoredValue(Value);
                    return true;
                } else {
                    bool Result = Set(Value, *this);
                    if (Result && isLastOperationSuccessful()) {
                        return Result;
                    } else {
                        ErrorMessage("Error in " + Name + ".SetValue(): " + Error);
                        return false;
                    }
                }
            } else {
                Error = "Feature can not be set";
            }
        } else {
            Error = "Feature is not Enabled";
        }
    } else {
        Error = "Feature is not Implemented";
    }
    ErrorMessage("Error in " + Name + ".SetValue(): " + Error);
    return false;
}
template<class T>
LOCAL_INLINE inline T PhoXiFeature<T>::GetStoredValue() const {
    return Value;
}
template<class T>
LOCAL_INLINE PhoXiFeature<T> &PhoXiFeature<T>::operator=(const PhoXiFeature<T> &Other) {
    Value = Other.Value;
    Error = Other.Error;
    Enabled = Other.Enabled;
    Initiated = Other.Initiated;
    return *this;
}
template<class T>
LOCAL_INLINE bool PhoXiFeature<T>::operator=(const T &Value) {
    return SetValue(Value);
}
template<class T>
LOCAL_INLINE PhoXiFeature<T>::operator T() {
    return GetValue();
}
template<class T>
LOCAL_INLINE std::unique_ptr <PhoXiFeatureSetGet<T>> PhoXiFeature<T>::operator->() {
    return std::unique_ptr<PhoXiFeatureSetGet<T>>(new PhoXiFeatureSetGet<T>(GetValue(), this));
}
template<class T>
LOCAL_INLINE bool PhoXiFeature<T>::operator==(const T &Other) {
    return (T) (*this) == Other;
}
template<class T>
LOCAL_INLINE bool PhoXiFeature<T>::operator!=(const T &Other) {
    return !this->operator==(Other);
}
}
}
#undef DISABLE_LOCAL_INLINE

#endif //_PHOXI_FEATURE_HPP

