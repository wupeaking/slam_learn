#ifndef GTSAN_BUILD_HPP
#define GTSAN_BUILD_HPP

#include <iostream>
#include <vector>
#include <string>
#include "rust/cxx.h"
using namespace std;

// #ifdef __cplusplus
// extern "C"
// {
// #endif

typedef struct
{
    double x;
    double y;
    double z;
    double w;
} Quaterniond_t;

typedef struct
{
    double x;
    double y;
    double z;
} Vector3d_t;

int example();

// #ifdef __cplusplus
// }
// #endif

class ABC
{
public:
    ABC() {}
    virtual ~ABC() {}
    void print() const;
    void add(rust::Str str);
    void clear();
    rust::Str get(int index) const;

private:
    vector<rust::Str> m_strs;
};

#include <memory>
std::unique_ptr<ABC> new_abc();
#endif