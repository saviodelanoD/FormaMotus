#pragma once
#include <string_view>
namespace SpiderAbdomenPolicy {
inline float Weight(std::string_view name) {
    return name=="Tail1"?.60f:name=="Tail2"?.25f:name=="Tail3"?.15f:0.f;
}
constexpr float limitDegrees=12.f;
}

