
#ifndef XENCODER_H
#define XENCODER_H

#include <stdint.h>
#include <stdio.h>

class XEncoderState;

class XEncoder
{
public:
    XEncoder();
    ~XEncoder();

private:
    XEncoderState* state; // wrap object data in internal state to avoid importing x264.h in external api
};

#endif // XENCODER_H