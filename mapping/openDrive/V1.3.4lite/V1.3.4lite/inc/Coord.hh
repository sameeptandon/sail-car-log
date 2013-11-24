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
 *  file:       Coord.hh
 * ---------------------------------------------------
 *  purpose:	container for a co-ordinate
 * ---------------------------------------------------
 *  first edit:	05.07.2005 by M. Dupuis @ VIRES GmbH
 *  last mod.:  05.07.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_COORD_HH
#define _OPENDRIVE_COORD_HH

// ====== INCLUSIONS ======

namespace OpenDrive
{

class Coord
{
    public:
        /**
        * get distance of two co-ordinates
        * @param coord1 position 1
        * @param coord2 position 2
        * @return inertial distance
        */
        static double getDist( const Coord & coord1, const Coord & coord2 ); 
        
        /**
        * get 2d distance of two co-ordinates
        * @param coord1 position 1
        * @param coord2 position 2
        * @return inertial distance in xy-plane
        */
        static double getDist2d( const Coord & coord1, const Coord & coord2 ); 
        
        static const double Deg2Rad; 
        static const double Rad2Deg; 
    
    private:

        /**
        * co-ordinate values:
        *   x, y, z, heading, pitch, roll
        */
        double  mX;
        double  mY;
        double  mZ;
        double  mH;
        double  mP;
        double  mR;

    public:
        /**
        * primitive constructor
        */
        explicit Coord();

        /**
        * constructor
        *   @param  x   x-value
        *   @param  y   y-value
        *   @param  z   z-value
        *   @param  h   heading
        *   @param  p   pitch
        *   @param  r   roll
        */
        explicit Coord( const double & x, const double & y, const double & z,
                        const double & h = 0.0, const double & p = 0.0, const double & r = 0.0 );

        /**
        * destructor
        */
        virtual ~Coord();
        
        /**
        * overload operator for value assignment
        * @param rhs    coord for assignment
        */
        void operator= ( const Coord & rhs );
        
        /**
        * overload operator for scalar multiplication
        * @param rhs    scalar for multiplication
        */
        Coord operator* ( const double & rhs );
        
        /**
        * add two co-ordinates
        * @param rhs    second co-ordinate
        */
        Coord operator+ ( const Coord & rhs );
        
        /**
        * subtract two co-ordinates
        * @param rhs    second co-ordinate
        */
        Coord operator- ( const Coord & rhs );
        
        /**
        * increase co-ordinate
        * @param rhs    second co-ordinate
        */
        void operator+= ( const Coord & rhs );
        
        /**
        * increase co-ordinate
        * @param rhs    second co-ordinate
        */
        void operator-= ( const Coord & rhs );
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getX() const;
        
        /**
        * member access function
        *   @return component of the co-ordinate
        */
        const double & getY() const;
        
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
        * set values of all member variable
        *   @param xyz and hpr values
        */
        void set( const double & x, const double & y, const double & z,
                  const double & h = 0.0, const double & p = 0.0, const double & r = 0.0 );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setX( const double & value );
        
        /**
        * set value of member variable
        *   @param component of the co-ordinate
        */
        void setY( const double & value );
        
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
        * initialize co-ordinate
        */
        void init();

        /**
        * print a co-ordinaate to stderr
        */
        void print() const;

        /**
        * get the length of the 3d-vector of the first 3 components fo the coordinate
        * @return value of vector (mX, mY, mZ)
        */
        double getValue();

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_COORD_HH */
