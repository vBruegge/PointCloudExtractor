#include "Fuselage.hpp"
#include "IOHandler.hpp"

class FuselageFitter {
public:

    FuselageFitter(Fuselage& section);

    void circularFit();

    void ellipseFit();

    void superellipseFit();

private:
    Fuselage section;
    IOHandler io;

};