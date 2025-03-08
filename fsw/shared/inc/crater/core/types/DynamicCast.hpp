#pragma once

#include <functional>
#include <optional>
#include <type_traits>

namespace crt
{

template <typename Derived, typename Base>
static inline std::optional<std::reference_wrapper<std::remove_reference_t<Derived>>> dyn_cast(Base& base)
{
    std::remove_reference_t<Derived>* derived = dynamic_cast<std::remove_reference_t<Derived>*>(&base);
    if (derived != nullptr) {
        return *derived;
    } else {
        return std::nullopt;
    }
}

template <typename Derived, typename Base>
static inline std::optional<std::remove_reference_t<Derived>*> dyn_cast(Base* base)
{
    std::remove_reference_t<Derived>* derived = dynamic_cast<std::remove_reference_t<Derived>*>(base);
    if (derived != nullptr) {
        return derived;
    } else {
        return std::nullopt;
    }
}

}  // namespace crt
