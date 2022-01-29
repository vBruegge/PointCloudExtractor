#include "Fuselage.hpp"

class FuselageFitter {
public:

    FuselageFitter(Fuselage& section);

    void circularFit();

    void ellipseFit();

    void superellipseFit();

    FuselageParameter getFuselageParameter();

private:
    Fuselage section;

};