"""

*    Pymaps 0.9 
*    Copyright (C) 2007  Ashley Camba <stuff4ash@gmail.com> http://xthought.org
*
*    This program is free software; you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation; either version 2 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software
*    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Changes by Bill (not a javascript programmer@) Luitje 29 July 2009
* fixed default parameter use for lists
* fixed icon javascript generation. works!
* updated example code

Some getting started info here:
http://lycos.dropcode.net/gregarius/Lonely_Code/2008/12/04/Google_Maps_and_Django

@ and not much of a python programmer either.

"""
import simplejson, urllib

class Icon:
    '''Get/make marker icons at http://mapki.com/index.php?title=Icon_Image_Sets'''
    def __init__(self,id='icon'):
        self.id = id
        self.image = ""     #uses default Google Maps icon
        self.shadow = ""
##        self.image = "http://labs.google.com/ridefinder/images/mm_20_blue.png"
##        self.shadow = "http://labs.google.com/ridefinder/images/mm_20_shadow.png"
        self.iconSize = (12, 20)    # these settings match above icons
        self.shadowSize = (22, 20)
        self.iconAnchor = (6, 20)
        self.infoWindowAnchor = (5, 1)

        
class Map:
    def __init__(self,id="map",pointlist=None):
        self.id       = id    # div id        
        self.width    = "500px"  # map div width
        self.height   = "300px"  # map div height
        self.center   = (29.71984,-95.398087)     # center map latitude coordinate
        self.zoom        = "1"   # zoom level
        self.navcontrols  =   True   # show google map navigation controls
        self.mapcontrols  =   True   # show toogle map type (sat/map/hybrid) controls
        if pointlist == None:
            self.points = []   # empty point list
        else:
            self.points = pointlist   # supplied point list
    
    def __str__(self):
        return self.id
        
    
    def setpoint(self, point):
        """ Add a point (lat,long,html,icon) """
        self.points.append(point)

class PyMap:
    """
    Python wrapper class for Google Maps API.
    """
    
    def __str__(self):
        return "Pymap"
    
    def __init__(self, key=None, maplist=None, iconlist=None):
        """ Default values """
        self.key      = "ABQIAAAAQQRAsOk3uqvy3Hwwo4CclBTrVPfEE8Ms0qPwyRfPn-DOTlpaLBTvTHRCdf2V6KbzW7PZFYLT8wFD0A"      # set your google key
        if maplist == None:
            self.maps = [Map()]
        else:
            self.maps = maplist
        if iconlist == None:
            self.icons = [Icon()]
        else:
            self.icons = iconlist
    
    def addicon(self,icon):
        self.icons.append(icon)
        
    def _navcontroljs(self,map):
        """ Returns the javascript for google maps control"""    
        if map.navcontrols:
            return  "           %s.gmap.addControl(new GSmallMapControl());\n" % (map.id)
        else:
            return ""    
    
    
    def _mapcontroljs(self,map):
        """ Returns the javascript for google maps control"""    
        if map.mapcontrols:
            return  "           %s.gmap.addControl(new GMapTypeControl());\n" % (map.id)
        else:
            return ""     
    
    
    def _showdivhtml(self,map):
        """ Returns html for dislaying map """
        html = """\n<div id=\"%s\">\n</div>\n""" % (map.id)
        return html
    
    def _point_hack(self, points):
        count = 1
        
        for item in points:
            open = str(item).replace("(", "[")
            open = open.replace(")", "]")
        
        return open
    
    
    def _mapjs(self,map):
        js = "%s_points = %s;\n" % (map.id,map.points)
        
        js = js.replace("(", "[")
        js = js.replace(")", "]")
        js = js.replace("u'", "'")
        js = js.replace("''","")    #python forces you to enter something in a list, so we remove it here
##        js = js.replace("'icon'", "icon")
        for icon  in self.icons:
            js = js.replace("'"+icon.id+"'",icon.id)
        js +=   """             var %s = new Map('%s',%s_points,%s,%s,%s);
        \n\n%s\n%s""" % (map.id,map.id,map.id,map.center[0],map.center[1],map.zoom, self._mapcontroljs(map), self._navcontroljs(map))
        return js
    
    
    
    def _iconjs(self,icon):
        js = """ 
                var %s = new GIcon(); 
                %s.image = "%s";
                %s.shadow = "%s";
                %s.iconSize = new GSize(%s, %s);
                %s.shadowSize = new GSize(%s, %s);
                %s.iconAnchor = new GPoint(%s, %s);
                %s.infoWindowAnchor = new GPoint(%s, %s);
        """ % (icon.id, icon.id, icon.image, icon.id, icon.shadow, icon.id, icon.iconSize[0],icon.iconSize[1],icon.id, icon.shadowSize[0], icon.shadowSize[1], icon.id, icon.iconAnchor[0],icon.iconAnchor[1], icon.id, icon.infoWindowAnchor[0], icon.infoWindowAnchor[1])
        return js
     
    def _buildicons(self):
        js = ""
        if (len(self.icons) > 0):
            for i in self.icons:
               js = js + self._iconjs(i)    
        return js
    
    def _buildmaps(self):
        js = ""
        for i in self.maps:
            js = js + self._mapjs(i)+'\n'
        return js

    def pymapjs(self):
        """ Returns complete javacript for rendering map """
        
        self.js = """\n<script src=\"http://maps.google.com/maps?file=api&amp;v=2&amp;key=%s\" type="text/javascript"></script>
        <script type="text/javascript">
        //<![CDATA[
        function load() {
            if (GBrowserIsCompatible()) {
                
            
            function Point(lat,long,html,icon) {
                  this.gpoint = new GMarker(new GLatLng(lat,long),icon);
                  this.html = html;
                  
               }               
               
               
               function Map(id,points,lat,long,zoom) {
                  this.id = id;
                  this.points = points;
                  this.gmap = new GMap2(document.getElementById(this.id));
                  this.gmap.setCenter(new GLatLng(lat, long), zoom);
                  this.markerlist = markerlist;
                  this.addmarker = addmarker;
                  this.array2points = array2points;
                   
                  function markerlist(array) {
                     for (var i in array) {
                        this.addmarker(array[i]);
                     }
                  }
                  
                  function array2points(map_points) {            
                      for (var i in map_points) {  
                        points[i] = new Point(map_points[i][0],map_points[i][1],map_points[i][2],map_points[i][3]);         }
                      return points;   
                    }                  
                  
                  function addmarker(point) {
                     if (point.html) {
                       GEvent.addListener(point.gpoint, "click", function() { // change click to mouseover or other mouse action
                           point.gpoint.openInfoWindowHtml(point.html);
                        
                       });
                       
                     }
                     this.gmap.addOverlay(point.gpoint);  
                  }
                  this.points = array2points(this.points);
                  this.markerlist(this.points);
            }  
                    %s
                    %s
            }
        }
        //]]>
        </script>
        
        
        """ % (self.key, self._buildicons(),self._buildmaps())
        return self.js 
    
        
def showhtml2(startCoord, endCoord):    
    html2 = """
<!DOCTYPE html>
<html>
  <head>
    <meta name="viewport" content="initial-scale=1.0, user-scalable=no">
    <meta charset="utf-8">
    <title>Directions service (complex)</title>
    <style>
      html, body, #map-canvas {{
        height: 100%;
        margin: 0px;
        padding: 0px
      }}
      #panel {{
        position: absolute;
        top: 5px;
        left: 50%;
        margin-left: -180px;
        z-index: 5;
        background-color: #fff;
        padding: 5px;
        border: 1px solid #999;
      }}
    </style>
    <script src="https://maps.googleapis.com/maps/api/js?v=3.exp&sensor=false"></script>
    <script>
var map;
var directionsDisplay;
var directionsService;
var stepDisplay;
var markerArray = [];

function initialize() {{
  // Instantiate a directions service.
  directionsService = new google.maps.DirectionsService();

  // Create a map and center it on Manhattan. 
  var mapCenter = new google.maps.LatLng(29.71984, -95.398087);
  var mapOptions = {{
    zoom: 13,
    center: mapCenter
  }}
  map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);

  // Create a renderer for directions and bind it to the map.
  var rendererOptions = {{
    map: map
  }}
  directionsDisplay = new google.maps.DirectionsRenderer(rendererOptions)

  // Instantiate an info window to hold step text.
  stepDisplay = new google.maps.InfoWindow({{
    disableAutoPan: false
    }});
}}

function calcRoute() {{

  
  // First, remove any existing markers from the map.
  for (var i = 0; i < markerArray.length; i++) {{
    markerArray[i].setMap(null);
  }}

  // Now, clear the array itself.
  markerArray = [];

  // Retrieve the start and end locations and create
  // a DirectionsRequest using WALKING directions.
  var start = "{0}" //"29.71984,-95.398087" //document.getElementById('start').value;
  var end = "{1}" //"29.732395,-95.394824" //document.getElementById('end').value;
  var request = {{
      origin: start,
      destination: end,
      travelMode: google.maps.TravelMode.WALKING
  }};

  // Route the directions and pass the response to a
  // function to create markers for each step.
  directionsService.route(request, function(response, status) {{
    if (status == google.maps.DirectionsStatus.OK) {{
      var warnings = document.getElementById('warnings_panel');
      warnings.innerHTML = '<b>' + response.routes[0].warnings + '</b>';
      directionsDisplay.setDirections(response);
      showSteps(response);
    }}
  }});
}}

function showSteps(directionResult) {{
  // For each step, place a marker, and add the text to the marker's
  // info window. Also attach the marker to an array so we
  // can keep track of it and remove it when calculating new
  // routes.
  
  var myRoute = directionResult.routes[0].legs[0];

  for (var i = 0; i < myRoute.steps.length; i++) {{
    var marker = new google.maps.Marker({{
      position: myRoute.steps[i].start_location,
      map: map
    }});
    attachInstructionText(marker, myRoute.steps[i].instructions);
    markerArray[i] = marker;
  }}
}}

function attachInstructionText(marker, text) {{
  google.maps.event.addListener(marker, 'click', function() {{
    // Open an info window when the marker is clicked on,
    // containing the text of the step.
    stepDisplay.setContent(text);
    stepDisplay.open(map, marker);
  }});
}}

google.maps.event.addDomListener(window, 'load', initialize);
//window.onload = calcRoute;

    </script>
  </head>
  <body onLoad="calcRoute()">

    <div id="map-canvas"></div>
    &nbsp;
    <div id="warnings_panel" style="width:100%;height:10%;text-align:center"></div>
  </body>
</html>
""".format(startCoord, endCoord)
    return html2

	
def generateHTML(startLat, startLong, destLat, destLong):
    import sys
    startCoord = "%0.5f,%0.5f"%(startLat, startLong) #"29.71984,-95.398087"
    endCoord = "%0.5f,%0.5f"%(destLat, destLong) #"29.732395,-95.394824"
    ##g = PyMap()                         # creates an icon & map by default
    ##icon2 = Icon('icon2')               # create an additional icon
    #icon2.image = "http://labs.google.com/ridefinder/images/mm_20_blue.png" # for testing only!
    #icon2.shadow = "http://labs.google.com/ridefinder/images/mm_20_shadow.png" # do not hotlink from your web page!
    #g.addicon(icon2)
    #g.key = "ABQIAAAAQQRAsOk3uqvy3Hwwo4CclBTrVPfEE8Ms0qPwyRfPn-DOTlpaLBTvTHRCdf2V6KbzW7PZFYLT8wFD0A" # you will get your own key
    #g.maps[0].zoom = 5
    #q = [1,1]                           # create a marker with the defaults
    #r = [2,2,'','icon2']                # icon2.id, specify the icon but no text
    #s = [3,3,'hello, <u>world</u>']     # don't specify an icon & get the default
    #g.maps[0].setpoint(q)               # add the points to the map
    #g.maps[0].setpoint(r)
    #g.maps[0].setpoint(s)
    
    #open('test.htm','wb').write(showhtml2(startCoord, endCoord))   # generate test file
    return showhtml2(startCoord, endCoord)

def getWaypoints():
    url="""
http://maps.googleapis.com/maps/api/directions/json?origin=29.71984,-95.398087&destination=29.732395,-95.394824&waypoints=Joplin,MO|Oklahoma+City,OK&sensor=false
"""
    dirsResult=simplejson.dumps(simplejson.load(urllib.urlopen(url)))
    return dirsResult
    
