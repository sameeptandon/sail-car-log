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
 *  file:       TrackCoord.hh
 * ---------------------------------------------------
 *  purpose:	co-ordinate handling for track system
 * ---------------------------------------------------
 *  first edit:	05.07.2005 by M. Dupuis @ VIRES GmbH
 *  last mod.:  28.01.2007 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_TRACKCOORD_HH
#define _OPENDRIVE_TRACKCOORD_HH

// ====== INCLUSIONS ======

namespace OpenDrive
{

class TrackCoord
{
    protected:

        /**
        * co-ordinate values (geometry)
        *   s, t, z, heading
        */
        int     mTrackId;
        double  mS;
        double  mT;
        double  mZ;
        double  mH;
        double  mP;   // added, Marius 28.01.2007
        double  mR;   // added, Marius 28.01.2007

    public:
        /**
        * primitive constructor
        */
        explicit TrackCoord();

        /**
        * constructor
        *   @param  track   track ID
        *   @param  s       longitudinal
        *   @param  t       lateral
        *   @param  z       z-value
        *   @param  h       heading
        *   @param  p       pitch
        *   @param  r       roll
        */
        explicit TrackCoord( const int & track,
                             const double & s, const double & t, 
                             const double & z = 0.0, const double & h = 0.0,
                             const double & p = 0.0, const double & r = 0.0 );

        /**
        * destructor
        */
        virtual ~TrackCoord();
        
        /**
        * overload operator for value assignment
        * @param rhs    coord for assignment
        */
        void operator= ( const TrackCoord & rhs );

        /**
        * add a track co-ordinate to the one
        * @param rhs    co-ordinate to be added
        **/
        void operator+= ( const TrackCoord & rhs );
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const int & getTrackId() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getS() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getT() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getZ() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getH() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getP() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getR() const;
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setTrackId( const int & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setS( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setT( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setZ( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setH( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setP( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setR( const double & value );
        
        /**
        * initialize track position
        */
        virtual void init();
        
        /**
        * print a co-ordinaate to stderr
        */
        virtual void print() const;

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_TRACKCOORD_HH */
