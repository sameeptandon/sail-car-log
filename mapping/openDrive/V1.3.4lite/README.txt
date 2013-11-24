/*
 * Copyright 2007 VIRES Simulationstechnologie GmbH
 * ALL RIGHTS RESERVED
 *
 * This softwre ("Software") was originally derived from a
 * code base owned by VIRES Simulationstechnologie GmbH ("VIRES")
 * 
 * LICENSE: VIRES grants the user ("Licensee") permission to reproduce,
 * distribute, and create derivative works from this Software,
 * provided that: (1) the user reproduces this entire notice within
 * both source and binary format redistributions and any accompanying
 * materials such as documentation in printed or electronic format;
 * (2) the Software is not to be used, or ported or modified for
 * use, except in conjunction with the VIRES OdrManager; and (3) the
 * names of VIRES Simulationstechnologie GmbH and VIRES may not be used
 * in any advertising or publicity relating to the Source Code without the
 * prior written permission of VIRES. No further license or permission
 * may be inferred or deemed or construed to exist with regard to the
 * Source Code or the code base of which it forms a part. All rights
 * not expressly granted are reserved.
 * 
 * This Software is provided to Licensee AS IS, without any
 * warranty of any kind, either express, implied, or statutory,
 * including, but not limited to, any warranty that the Software
 * will conform to specifications, any implied warranties of
 * merchantability, fitness for a particular purpose, and freedom
 * from infringement, and any warranty that the documentation will
 * conform to the program, or any warranty that the Software will
 * be error free.
 * 
 * IN NO EVENT WILL VIRES BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT
 * LIMITED TO DIRECT, INDIRECT, SPECIAL OR CONSEQUENTIAL DAMAGES,
 * ARISING OUT OF, RESULTING FROM, OR IN ANY WAY CONNECTED WITH THE
 * SOFTWARE, WHETHER OR NOT BASED UPON WARRANTY, CONTRACT, TORT OR
 * OTHERWISE, WHETHER OR NOT INJURY WAS SUSTAINED BY PERSONS OR
 * PROPERTY OR OTHERWISE, AND WHETHER OR NOT LOSS WAS SUSTAINED FROM,
 * OR AROSE OUT OF USE OR RESULTS FROM USE OF, OR LACK OF ABILITY TO
 * USE, THE SOFTWARE.
 * 
 * VIRES Simulationstechnologie GmbH, Grassinger Strasse 8, 83043 Bad Aibling
 * Germany, opendrive@vires.com, www.vires.com
 */

OpenDRIVE Manager 1.3, Lite Edition, August 05, 2012
------------------------------------------------------

WHAT IS IT?
-----------
The OpenDRIVE Manager is a toolkit library for the handling and
evaluation of OpenDRIVE databases. It is highly optimized for real-time
requirements so that you can make sure you don't spend more time than
absolutely necessary to retrieve information from a road database.

Typical applications include 
    - calculation of contact point information for driving dynamics
    - traffic simulation
    - visualization of road scenarios
    - etc.

The software is distributed as a library which can easily be linked
to your application.

This software package contains a lite edition of the OpenDRIVE
Manager by VIRES Simulationstechnologie GmbH, Germany.


WHAT CAN I DO WITH IT?
----------------------
The lite edition will enable you to read an OpenDRIVE database and perform 
various co-ordinate conversions between inertial and track-based positions.

You will also be able to query the curvature at a given track position, 
retrieve information about the track (road) length, query the width
of a lane and print the content of the OpenDRIVE database after it has
been read to memory.


WHICH CO-ORDINATES ARE AVAILABLE?
----------------------------------
Three position types are supported:

Inertial Position (see Coord.hh):
        x
        y
        z
        heading
        pitch
        roll
                
Track Position (see TrackCoord.hh):
        track ID (i.e. road ID)
        s
        t
                
Lane Position (see LaneCoord.hh):
        track ID (i.e. road ID)
        s
        lane ID
        t-offset within lane
        
        
Further components of the co-ordinates may be listed in the respective
header files but are not supported by the lite edition of the OpenDRIVE Manager.


HOW DO I USE IT?
----------------
An example is included in the file 

    src/main.cc

You will find a series of comments in the source code. They will help you
understand what it's doing. 

The evaluation library is made available for a whole series of targets:
- Linux, 32bit              lib/Linux/i586
- Linux, 64bit              lib/Linux/x86-64
- Windows, VS2010, 32bit    lib/Windows/VS2010-32/Debug
                            lib/Windows/VS2010-32/DynamicDebug
                            lib/Windows/VS2010-32/DynamicRelease
                            lib/Windows/VS2010-32/Release
- Windows, VS2010, 64bit    lib/Windows/VS2010-64/Debug
                            lib/Windows/VS2010-64/DynamicDebug
                            lib/Windows/VS2010-64/DynamicRelease
                            lib/Windows/VS2010-64/Release

    
Compile the example and link it to the enclosed library <libODriveLite.1.3.4.a>

You can do this from the command line (here: 32bit Linux machine) with

    g++ -m32 -o odrTest src/main.cc -Iinc -Llib/Linux/i586 -lODriveLite.1.3.4

or use your own compilation environment.

Run your executable with the enclosed sample database <sample1.1.xodr>, 
which is also available as free download from the OpenDRIVE website.

    odrTest data/sample1.1.xodr
       
You will see quite a few lines of console output on the screen. Make sure your
console's history is large enough or pipe the output (from stderr) into a file
which you can easily parse with a text editor.


WHICH RESTRICTIONS APPLY TO THE LITE EDITION?
---------------------------------------------
You may use the lite edition for evaluation purposes regarding the OpenDRIVE 
data format and our real-time ODrive library. You must not use the library for 
commercial purposes other than the evaluation.

In order to distribute or sell any software using our high-performance 
algorithms, you need to purchase a full license of the OpenDRIVE Manager. 

This will not only legalize the re-distribution of the software but also 
bring you a vast number of additional features including but not limited to:
    - unlimited number of contact points
    - further position conversion functions
    - query of signals and other road features
    - direct access to each OpenDRIVE node in memory
    - class headers for OpenDRIVE nodes
    - support from VIRES experts


MAY I ASK A QUESTION?
---------------------
The lite edition of the OpenDRIVE Manager does not come with any further
support. Some advice and tricks may be available via the OpenDRIVE website

    www.opendrive.org
    
or on

    www.vires.com
    
    
However if you experience serious trouble, please let us know via 

    opendrive@vires.com
    


YES I LIKE IT! WHERE CAN I PURCHASE THE FULL VERSION?
-----------------------------------------------------
We knew you'd like it ;-) The OpenDRIVE Manager is available in various 
license schemes. Please contact us via 

    opendrive@vires.com 
    
for your individual quotation..


   
   
August 05, 2012

Marius Dupuis
    





