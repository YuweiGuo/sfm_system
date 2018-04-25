/* ######################################################################### */
/* \file VirtualBundleAdjustor.h
 * \project name : SfMLib
 * \class name : VirtualBundleAdjustor
 * \brief :     virtual class for the bundle adjustor
 *              we define a virtual function for the subclass
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */

#ifndef VIRTUALBUNDLEADJUSTOR_H
#define VIRTUALBUNDLEADJUSTOR_H

#include "SfMLibObject.h"

class VirtualBundleAdjustor : SfMLibObject
{
public:
    VirtualBundleAdjustor(){};
    virtual bool DoBundleAdjust() = 0;
};

#endif /* _AUTHOR_FILENAME_H */
/* ------------------------------------------------------------------------- */
/*
 * RIGHT OF USE. This document may neither be passed on to third parties or
 * reproduced nor its contents utilized or divulged without the expressed
 * prior permission of the Institute for Real-Time Learning Systems. In case of
 * contravention, the offender shall be liable for damages.
 *
 *
 * COPYRIGHT (C) Institut für Echtzeit Lernsysteme, Prof. K.-D. Kuhnert 2007-2015
 *
 */
/* ------------------------------------------------------------------------- */
