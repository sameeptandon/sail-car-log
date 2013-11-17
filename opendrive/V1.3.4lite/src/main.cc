/*
 * Copyright 2007 VIRES Simulationstechnologie GmbH
 * ALL RIGHTS RESERVED
 *
 * This source code ("Source Code") was originally derived from a
 * code base owned by VIRES Simulationstechnologie GmbH ("VIRES")
 * 
 * LICENSE: VIRES grants the user ("Licensee") permission to reproduce,
 * distribute, and create derivative works from this Source Code,
 * provided that: (1) the user reproduces this entire notice within
 * both source and binary format redistributions and any accompanying
 * materials such as documentation in printed or electronic format;
 * (2) the Source Code is not to be used, or ported or modified for
 * use, except in conjunction with the VIRES OdrManager; and (3) the
 * names of VIRES Simulatonstechnologie GmbH and VIRES may not be used
 * in any advertising or publicity relating to the Source Code without the
 * prior written permission of VIRES. No further license or permission
 * may be inferred or deemed or construed to exist with regard to the
 * Source Code or the code base of which it forms a part. All rights
 * not expressly granted are reserved.
 * 
 * This Source Code is provided to Licensee AS IS, without any
 * warranty of any kind, either express, implied, or statutory,
 * including, but not limited to, any warranty that the Source Code
 * will conform to specifications, any implied warranties of
 * merchantability, fitness for a particular purpose, and freedom
 * from infringement, and any warranty that the documentation will
 * conform to the program, or any warranty that the Source Code will
 * be error free.
 * 
 * IN NO EVENT WILL VIRES BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT
 * LIMITED TO DIRECT, INDIRECT, SPECIAL OR CONSEQUENTIAL DAMAGES,
 * ARISING OUT OF, RESULTING FROM, OR IN ANY WAY CONNECTED WITH THE
 * SOURCE CODE, WHETHER OR NOT BASED UPON WARRANTY, CONTRACT, TORT OR
 * OTHERWISE, WHETHER OR NOT INJURY WAS SUSTAINED BY PERSONS OR
 * PROPERTY OR OTHERWISE, AND WHETHER OR NOT LOSS WAS SUSTAINED FROM,
 * OR AROSE OUT OF USE OR RESULTS FROM USE OF, OR LACK OF ABILITY TO
 * USE, THE SOURCE CODE.
 * 
 * VIRES Simulationstechnologie GmbH, Grassinger Strasse 8, 83043 Bad Aibling
 * Germany, opendrive@vires.com, www.vires.com
 */
 
/* ===================================================
 *  file:       main.cc
 * ---------------------------------------------------
 *  purpose:    main program for demonstrating the
 *              OdrManagerLite
 * ---------------------------------------------------
 *  first edit: 08.07.2006 by M. Dupuis @ VIRES GmbH
 *  last mod.:  02.02.2007 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */

/* ====== INCLUSIONS ====== */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "OdrManagerLite.hh"

/* ====== GLOBAL VARIABLES ====== */

int main( int argc, char **argv ) 
{
    OpenDrive::OdrManagerLite myManager;

    // has it been started with "odrMgr <filename>"?
    if ( argc < 2 ) {
        std::cerr << "Usage: " << std::endl;
        std::cerr << "   odrMgr <filename>" << std::endl;
        exit( -1 );
    }

    std::cerr << "main: analyzing file <" << argv[1] << ">" << std::endl;
    
    if ( !myManager.loadFile( argv[1] ) ) 
    {
        std::cerr << "main: could not load <" << argv[1] << ">" << std::endl;
        exit( 0 );
    }
    
    /********************************************************************
    *                         NOTE
    * The following samples will only produces reasonable
    * results when being used with the publicly available OpenDRIVE
    * sample database (check the www.opendrive.org for download)
    *********************************************************************/
    
    /********************************************************************
    *                         SAMPLE no. 1
    *                    print the data tree
    *********************************************************************/
    // print the contents of the internal data tree after the file
    // has been successfully read to memory
    std::cerr << "main: printing data contents" << std::endl;
    myManager.printData();
    
    /********************************************************************
    *                         SAMPLE no. 2
    *             navigate on the road data, a few samples
    *********************************************************************/
    std::cerr << "main: position tests" << std::endl;
    
    // the upcoming tests all require a position object which holds
    // inputs and results for conversions between the different position
    // types (inertial, lane, track) which are available in the library
    
    // with the OdrManagerLite, only one position object can be managed
    // within a process meaning that only one contact point will be available
    // for high-performance computations
    
    // create a position object for access to the data
    // this should be done for each contact point (only one is
    // supported in OdrManagerLite)
    OpenDrive::Position* myPos= myManager.createPosition();
    
    // activate the previously created position object for further
    // calculations
    myManager.activatePosition( myPos );
    
    
    /********************************************************************
    *                         SAMPLE no. 2.0
    *             convert between track and inertial co-ordinates
    *********************************************************************/

    for ( double s = 0.0; s < 4600.0; s+= 100.0 ) 
    {
        // set the track position to track no. 3, running s value and t = 3.0m
        // i.e. 3.0m left of the centerline
        myManager.setTrackPos( 3, s, 3.0 );
        
        // convert the track co-ordinate into the corresponding inertial co-ordinate
        bool result = myManager.track2inertial();
        
        // if successful the result is found in the inertial position stored in the manager
        fprintf( stderr, "testCase 0: result = %d, s = %.3f, x = %.8lf, y = %.8lf, z = %.8lf, pitch=%.8lf\n", result, s, 
                          myManager.getInertialPos().getX(), myManager.getInertialPos().getY(), 
                          myManager.getInertialPos().getZ(), myManager.getInertialPos().getP() );
       
        // Now the other way round: use the current inertial position and convert it into a track position.
        // This should result in the original track co-ordinate. Note that the inertial position is not altered
        result = myManager.inertial2track();
        
        fprintf( stderr, "testCase 0: result = %d, x = %.8lf, y = %.8lf, track = %d, s = %.8lf, t = %.8lf\n", 
                          result, myManager.getInertialPos().getX(), myManager.getInertialPos().getY(),
                          myManager.getTrackPos().getTrackId(), myManager.getTrackPos().getS(), myManager.getTrackPos().getT() );
    }

    /********************************************************************
    *                         SAMPLE no. 2.1
    *             convert between lane and inertial co-ordinates
    *********************************************************************/

    for ( double s = 0.0; s < 1300.0; s+= 20.0 ) 
    {
        // set the lane position to track no. 5, lane 1, running s value and
        // 0.0m offset from the lane center
        myManager.setLanePos( 5, 1, s, 0.0 );
        
        // convert the lane co-ordinate into the corresponding inertial co-ordinate
        bool result = myManager.lane2inertial();
        
        fprintf( stderr, "testCase 1: result = %d, s = %.3f, x = %.8lf, y = %.8lf, z = %.8lf, pitch=%.8lf, hdg=%.8lf, roll=%.8lf, curv=%.8lf\n", 
                          result, s, 
                          myManager.getInertialPos().getX(), myManager.getInertialPos().getY(), 
                          myManager.getInertialPos().getZ(), myManager.getInertialPos().getP(),
                          myManager.getInertialPos().getH(), myManager.getInertialPos().getR(), myManager.getCurvature() );
       
        // Now the other way round: use the current inertial position and convert it into a lane position.
        // This should result in the original lane co-ordinate. Note that the inertial position is not altered
        result = myManager.inertial2lane();
        
        fprintf( stderr, "testCase 1: result = %d, x = %.8lf, y = %.8lf, track = %d, lane = %d, s = %.8lf, offset = %.8lf, curv=%.8lf\n", 
                          result, myManager.getInertialPos().getX(), myManager.getInertialPos().getY(),
                          myManager.getLanePos().getTrackId(), myManager.getLanePos().getLaneId(), 
                          myManager.getLanePos().getS(), myManager.getLanePos().getOffset(),
                          myManager.getCurvature() );
    }
    
    /********************************************************************
    *                         SAMPLE no. 2.2
    *             convert between inertial and lane co-ordinate
    *             at two different discrete positions (slight difference
    *             to sample no. 2.1
    *********************************************************************/

    // create an inertial xy-position and assign it to the current position
    OpenDrive::Coord myCoord( 1667.0, 847.0, 0.0 );
    myManager.setPos( myCoord );
                    
    int result = myManager.inertial2lane();
    
    // Only x and y of the stored inertial position are used to project the position onto the track
    // (in inertial z-direction). The result is a lane position and an inertial footpoint on the track. 
    // The footpoint will most probably not be identical to the original inertial position (which e.g. may
    // be above the track)
    // In most applications however, the inertial2lane() or inertial2track() conversion is being used 
    // for projecting an object's inertial xy/position onto the track in order to use the resulting footpoint as
    // the new inertial position. The user actually "slaves" the position (and the depending object like a car
    // or a wheel) to the track. The footpoint's z-value is identical to the track's inertial z-value, the
    // footpoint's heading, pitch and roll values are identical to the track's inertial hpr values.
    
    // copy the resulting footpoint to the inertial position (i.e. slave and align to track)
    myManager.footPoint2inertial();
                        
    fprintf( stderr, "testCase 2: result = %d, x = %.8lf, y = %.8lf, track = %d, t = %.8lf, lane = %d, s = %.8lf, offset = %.8lf, width = %.8lf, curv=%.8lf, hdg=%.8lf\n", 
                      result, myManager.getInertialPos().getX(), myManager.getInertialPos().getY(),
                      myManager.getLanePos().getTrackId(), myManager.getLanePos().getT(), myManager.getLanePos().getLaneId(), 
                      myManager.getLanePos().getS(), myManager.getLanePos().getOffset(),
                      myManager.getLaneWidth(), myManager.getCurvature(), myManager.getInertialPos().getH() );
    
    
    // same procedure for another point in the database
    myCoord = OpenDrive::Coord( 1083.0, 1550.0, 0.0 );
    myManager.setPos( myCoord );
    
    result = myManager.inertial2lane();
    myManager.footPoint2inertial();
        
    fprintf( stderr, "testCase 2: result = %d, x = %.8lf, y = %.8lf, track = %d, t = %.8lf, lane = %d, s = %.8lf, offset = %.8lf, width = %.8lf, curv=%.8lf, hdg=%.8lf\n", 
                      result, myManager.getInertialPos().getX(), myManager.getInertialPos().getY(),
                      myManager.getLanePos().getTrackId(), myManager.getLanePos().getT(), myManager.getLanePos().getLaneId(), 
                      myManager.getLanePos().getS(), myManager.getLanePos().getOffset(),
                      myManager.getLaneWidth(), myManager.getCurvature(), myManager.getInertialPos().getH() );
     return 1;
}

