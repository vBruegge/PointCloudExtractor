#ifndef FUSELAGEFITTER_H
#define FUSELAGEFITTER_H

#include "Fuselage.hpp"
#include "IOHandler.hpp"

class FuselageFitter {
public:

    FuselageFitter(Fuselage& section_);

    void circularFit();

    void ellipseFit();

    FuselageParameter superellipseFit();

private:
    Fuselage section;
    IOHandler io;

};

#endif