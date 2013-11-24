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
 *  file:       OpenDRIVE.hh
 * ---------------------------------------------------
 *  purpose:	definitions for OpenDRIVE
 * ---------------------------------------------------
 *  first edit:	10.11.2005 by M. Dupuis @ VIRES GmbH
 *  last mod.:  21.06.2007 by M. Knerr @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_HH
#define _OPENDRIVE_HH

namespace OpenDrive
{
    // OPCODES
    enum EnOpcode
    {
        ODR_OPCODE_NONE =  0,
        ODR_OPCODE_HEADER,
        ODR_OPCODE_ROAD_HEADER,
        ODR_OPCODE_ROAD_LINK,
        ODR_OPCODE_ROAD_TYPE,
        ODR_OPCODE_GEO_HEADER,
        ODR_OPCODE_GEO_LINE,
        ODR_OPCODE_GEO_SPIRAL,
        ODR_OPCODE_GEO_ARC,
        ODR_OPCODE_ELEVATION,
        ODR_OPCODE_LANE_SECTION,
        ODR_OPCODE_LANE,
        ODR_OPCODE_LANE_LINK,
        ODR_OPCODE_LANE_WIDTH,
        ODR_OPCODE_LANE_MATERIAL,
        ODR_OPCODE_LANE_VISIBILITY,
        ODR_OPCODE_SIGNAL,
        ODR_OPCODE_LANE_VALIDITY,
        ODR_OPCODE_SIGNAL_DEPEND,
        ODR_OPCODE_CONTROLLER,
        ODR_OPCODE_CONTROL_ENTRY,
        ODR_OPCODE_JUNCTION_HEADER,
        ODR_OPCODE_JUNCTION_LINK,
        ODR_OPCODE_JUNCTION_PRIORITY,
        ODR_OPCODE_OBJECT,
        ODR_OPCODE_USER_DATA,
        ODR_OPCODE_JUNCTION_LANE_LINK,
        ODR_OPCODE_CROSSFALL,
        ODR_OPCODE_JUNCTION_CONTROL,
        ODR_OPCODE_LANE_ROAD_MARK,
        ODR_OPCODE_PREDECESSOR,
        ODR_OPCODE_SUCCESSOR,
        ODR_OPCODE_LANES,
        ODR_OPCODE_LANES_LEFT,
        ODR_OPCODE_LANES_CENTER,
        ODR_OPCODE_LANES_RIGHT,
        ODR_OPCODE_PLANVIEW,
        ODR_OPCODE_ELEV_PROFILE,
        ODR_OPCODE_LATERAL_PROFILE,
        ODR_OPCODE_OBJECTS,
        ODR_OPCODE_SIGNALS,
        ODR_OPCODE_OPENDRIVE,
        ODR_OPCODE_SUPERELEVATION,
        ODR_OPCODE_GEO_POLY,
        ODR_OPCODE_LANE_SPEED,
        ODR_OPCODE_LANE_ACCESS,
        ODR_OPCODE_LANE_HEIGHT,
        ODR_OPCODE_CORNER_INERTIAL,
        ODR_OPCODE_CORNER_ROAD,
        ODR_OPCODE_CORNER_RELATIVE,
        ODR_OPCODE_TUNNEL,
        ODR_OPCODE_BRIDGE,
        ODR_OPCODE_SIGNAL_REFERENCE,
        ODR_OPCODE_OBJECT_OUTLINE,
        ODR_OPCODE_SURFACE,
        ODR_OPCODE_SURFACE_CRG,
        ODR_OPCODE_TRAFFIC_OBJECT = 2000,
        ODR_OPCODE_LANE_OFFSET
    };

    // GENERAL TYPE
    enum EnType {
        ODR_TYPE_NONE = 0,
        ODR_TYPE_ROAD,
        ODR_TYPE_JUNCTION
    };

    // LINK TYPE
    enum EnLinkType {
        ODR_LINK_TYPE_PREDECESSOR = 1,
        ODR_LINK_TYPE_SUCCESSOR,
        ODR_LINK_TYPE_NEIGHBOR
    };

    // LINK POINT
    enum EnLinkPoint {
        ODR_LINK_POINT_START = 0,
        ODR_LINK_POINT_END
    };

    // ROAD TYPE
    enum EnRoadType {
        ODR_ROAD_TYPE_NONE = 0,
        ODR_ROAD_TYPE_RURAL,
        ODR_ROAD_TYPE_MOTORWAY,
        ODR_ROAD_TYPE_TOWN,
        ODR_ROAD_TYPE_LOW_SPEED,
        ODR_ROAD_TYPE_PEDESTRIAN
    };

    // GEOMETRY TYPE
    enum EnGeometryType {
        ODR_GEO_TYPE_LINE = 0,
        ODR_GEO_TYPE_SPIRAL,
        ODR_GEO_TYPE_ARC
    };

    // ROAD MARK
    enum EnRoadMark {
        ODR_ROAD_MARK_NONE = 0,
        ODR_ROAD_MARK_SOLID,
        ODR_ROAD_MARK_SOLID_BOLD,
        ODR_ROAD_MARK_BROKEN,
        ODR_ROAD_MARK_SOLID_DOUBLE,
        ODR_ROAD_MARK_BROKEN_BOLD,
        ODR_ROAD_MARK_SOLID_YELLOW,
        ODR_ROAD_MARK_SOLID_DOUBLE_YELLOW
    };

    enum EnRoadMarkType {
        ODR_ROAD_MARK_TYPE_NONE=  0,
        ODR_ROAD_MARK_TYPE_SOLID,
        ODR_ROAD_MARK_TYPE_BROKEN,
        ODR_ROAD_MARK_TYPE_SOLID_SOLID,
        ODR_ROAD_MARK_TYPE_SOLID_BROKEN,
        ODR_ROAD_MARK_TYPE_BROKEN_SOLID
    };
    
    enum EnRoadMarkWeight {
        ODR_ROAD_MARK_WEIGHT_NONE = 0,
        ODR_ROAD_MARK_WEIGHT_STANDARD,
        ODR_ROAD_MARK_WEIGHT_BOLD
    };
    
    enum EnRoadMarkColor {
        ODR_ROAD_MARK_COLOR_NONE = 0,
        ODR_ROAD_MARK_COLOR_STANDARD,
        ODR_ROAD_MARK_COLOR_YELLOW,
        ODR_ROAD_MARK_COLOR_RED,
        ODR_ROAD_MARK_COLOR_WHITE
    };
    
    // LANE TYPE
    enum EnLaneType {
        ODR_LANE_TYPE_NONE = 0,
        ODR_LANE_TYPE_DRIVING,
        ODR_LANE_TYPE_STOP,
        ODR_LANE_TYPE_SHOULDER,
        ODR_LANE_TYPE_BIKING,
        ODR_LANE_TYPE_SIDEWALK,
        ODR_LANE_TYPE_BORDER,
        ODR_LANE_TYPE_RESTRICTED,
        ODR_LANE_TYPE_PARKING,
        ODR_LANE_TYPE_MWY_ENTRY,
        ODR_LANE_TYPE_MWY_EXIT,
        ODR_LANE_TYPE_SPECIAL1,
        ODR_LANE_TYPE_SPECIAL2,
        ODR_LANE_TYPE_SPECIAL3,
        ODR_LANE_TYPE_SPECIAL4,
        ODR_LANE_TYPE_DRIVING_ROADWORKS
    };

    // OBJECT TYPE
    enum EnObjectType {
        ODR_OBJECT_TYPE_OBSTACLE = 0,
        ODR_OBJECT_TYPE_WIND
    };
    
    // DIRECTIONS
    enum EnDirection {
        ODR_DIRECTION_PLUS = 0,
        ODR_DIRECTION_MINUS
    };
    
    // SIDES
    enum EnSide {
        ODR_SIDE_BOTH = 0,
        ODR_SIDE_LEFT,
        ODR_SIDE_RIGHT
    };
    
    // ACCESS RESTRICTION
    enum EnAccessRestriction {
        ODR_LANE_ACCESS_RESTRICT_NONE = 0,
        ODR_LANE_ACCESS_RESTRICT_SIMULATOR,
        ODR_LANE_ACCESS_RESTRICT_AUTONOMOUS,
        ODR_LANE_ACCESS_RESTRICT_PEDESTRIAN
    };
    
    // TUNNEL TYPES
    enum EnTunnelType {
        ODR_TUNNEL_NONE = 0,
        ODR_TUNNEL_STANDARD,
        ODR_TUNNEL_UNDERPASS
    };
    
    // BRIDGE TYPES
    enum EnBridgeType {
        ODR_BRIDGE_NONE = 0,
        ODR_BRIDGE_CONCRETE,
        ODR_BRIDGE_STEEL,
        ODR_BRIDGE_BRICK
    };

    // COUNTRY CODES
    enum EnCountryCode {
        ODR_COUNTRY_OPENDRIVE = 0,
        ODR_COUNTRY_USA = 1,
        ODR_COUNTRY_FRANCE = 33,
        ODR_COUNTRY_GERMANY = 49
    };

    // MATERIAL CODES
    enum EnMaterialCode {
        ODR_MATERIAL_DEFAULT = 0,
        ODR_MATERIAL_SNOW
    };

} // namespace OpenDRIVE

#endif /* _OPENDRIVE_HH */
