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
 * names of VIRES Simulationstechnologie GmbH and VIRES may not be used
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
 * VIRES Simulationstechnologie GmbH, Oberaustrasse 34, 83026 Rosenheim
 * Germany, opendrive@vires.com, www.vires.com
 */
/* ===================================================
 *  file:       OdrManagerLite.hh
 * ---------------------------------------------------
 *  purpose:	manager for the interaction with
 *              the OpenDRIVE routines, Lite edition
 *              (base class for full OdrManager)
 * ---------------------------------------------------
 *  first edit:	02.02.2007 by M. Dupuis @ VIRES GmbH
 *  last mod.:  13.06.2007 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ODRMANAGER_LITE_HH
#define _OPENDRIVE_ODRMANAGER_LITE_HH

// ====== INCLUSIONS ======
#include <string>
#include "Coord.hh"
#include "LaneCoord.hh"

namespace OpenDrive
{
    
// forward declarations of unpublished classes
class Position;
class RoadData;

class OdrManagerLite
{
    public:
        /**
        * default constructor
        */
        explicit OdrManagerLite();
        
        /**
        * default constructor
        * @flags used for VIRES internal purposes
        */
        explicit OdrManagerLite( const unsigned int & flags );

        /**
        * destructor
        */
        virtual ~OdrManagerLite();
        
        /**
        * show intro text
        */
        virtual void intro();

        /**
        * load a file
        * @param  name  filename (complete path)
        * @return success/failure
        */
        virtual bool loadFile( const std::string & name );
        
        /**
        * print the data contained in memory
        */
        void printData();
        
        /**
        * get a new position object
        * @return NULL if no object has been created
        */
        virtual Position* createPosition();
                
        /**
        * set the position object on which subsequent
        * calls shall be executed
        * @param pos position object which is to be made current
        */
        void activatePosition( Position* pos );

        /**
        * get the currently active track co-ordinate
        *   @param  pos position object on which to work
        *   @return track co-ordinate
        */
        const TrackCoord & getTrackPos() const;

        /**
        * member access function
        *   @return track co-ordinate
        */
        const LaneCoord & getLanePos() const;

        /**
        * member access function
        *   @return inertial (real-world) co-ordinate
        */
        const Coord & getInertialPos() const;

        /**
        * member access function
        *   @return foot point inertial (real-world) co-ordinate
        */
        const Coord & getFootPoint() const;

        /**
        * member access function
        *   @param set the track co-ordinate
        */
        void setPos( const TrackCoord & value );

        /**
        * explicitly set the track position
        * @param id track id
        * @param s  track co-ordinate
        * @param t  lateral co-ordinate
        */
        void setTrackPos( const int & id, const double & s, const double & t = 0.0 );
        void setTrackPos( const TrackCoord & value );

        /**
        * member access function
        *   @param set the lane co-ordinate
        */
        void setPos( const LaneCoord & value );

        /**
        * explicitly set the lane position
        * @param trackId track id
        * @param laneId   lane id
        * @param s  track co-ordinate
        * @param offset  lateral offset from lane center
        */
        void setLanePos( const int & trackId, const int & laneId, const double & s, const double & offset = 0.0 );
        void setLanePos( const LaneCoord & value );

        /**
        * member access function
        *   @param set the inertial co-ordinate
        */
        void setPos( const Coord & value );

        /**
        * explicitly set the inertial position
        * @param x  x co-ordinate
        * @param y  y co-ordinatte
        * @param z  z co-ordinate
        */
        void setInertialPos( const double & x, const double & y, const double & z );

        /**
        * convert track position into inertial position
        *   @return true if successful
        */
        bool track2inertial();

        /**
        * convert inertial position into track position and attach to track
        *   @return true if successful
        */
        bool inertial2track();

        /**
        * convert lane position into inertial position
        *   @return true if successful
        */
        bool lane2inertial();

        /**
        * convert inertial position into lane position and slave to track
        *   @return true if successful
        */
        virtual bool inertial2lane();

        /**
        * print position info
        * @param ident  number of leading spaces
        */
        void print( int ident = 0 );

        /**
        * get the curvature at current position
        * @return curvature
        */
        const double & getCurvature() const;

        /**
        * get the length of a given track
        * @param id of the track
        */
        double getTrackLen( int trackId );

        /**
        * member access function
        * @return  width of current lane
        */
        const double & getLaneWidth();

        /**
        * copy a foot point into the inertial position
        */
        void footPoint2inertial();
        
    protected:
        /**
        * singleton object holding the actual road data
        */
        RoadData* mRoadData;
        
        /**
        * current position object
        */
        Position *mPos;

        /** 
        * keep info about surface scale for road surface data
        */
        double mSurfaceScale;

};

} // namespace OpenDrive

#endif /* _OPENDRIVE_ODRMANAGER_LITE_HH */
