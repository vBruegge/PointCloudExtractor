#ifndef FUSELAGEFITTER_H
#define FUSELAGEFITTER_H

#include "Fuselage.hpp"
#include "IOHandler.hpp"

class FuselageFitter {
public:

    /**
     * @brief Construct a new Fuselage Fitter object
     * 
     * @param section_ fuselage section
     */
    FuselageFitter(Fuselage& section_, std::string sourceFolder);

    /**
     * @brief fits the saved fuselage cloud with a circle
     * 
     */
    void circularFit();

    /**
     * @brief fits the saved fuselage cloud with a ellipse
     * 
     */
    void ellipseFit();

    /**
     * @brief fits the saved fuselage cloud with a superellipse
     * 
     * @return FuselageParameter new fitted parameters (including epsilon)
     */
    FuselageParameter superellipseFit();

private:
    Fuselage section;
    IOHandler io;

};

#endif