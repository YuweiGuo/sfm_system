/* ######################################################################### */
/* \file SfMLibTriangulator.h
 * \project name : SfMLib
 * \class name : SfMLibTriangulator
 * \brief :     a implementation for virtual class.
 *              hier we define the output parameters for the class.
 *              and the corresponding operator.
 *              the virtual function is not implemented jet.
 *
 *              in this class we defined :
 *              1). output parameter _pnt_cloud
 *              2). the method to get it GetPointCloud
 *              3)- the method to store it StorePointCloud
 *
 * Copyright (c) 2015 EZLS
 *
 * \author Yuwei Guo
 * \version 2.0
 * \date 2017-1-1
 */
/* ######################################################################### */

#ifndef SFMLIBTRIANGULATOR_H
#define SFMLIBTRIANGULATOR_H

#include "BasicSetting.h"
#include "VirtualTriangulator.h"

class SfMLibTriangulator : VirtualTriangulator
{
public:
    SfMLibTriangulator(){}
private:
    std::vector<CloudPoint> _pnt_cloud;
public:
    virtual double DoTriangulation() = 0;
    void StorePointCloud(const std::vector<CloudPoint>& pnt_cloud);
    bool GetPointCloud(std::vector<CloudPoint>& pnt_cloud);
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

