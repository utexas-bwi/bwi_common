// Constants
var ROSBRIDGEPORT = 9090;
var MJPEGSERVERPORT = 8080;
var IMAGESERVERPORT = 8000;
var ERROR_NOTOURALLOWED = -2;
var ERROR_TOURINPROGRESS = -3;
var ERROR_NOTTOURLEADER = -4;
var ERROR_NOTOURINPROGRESS = -5;

// Settings
var VIDEO_QUALITY = 30;

// Globals
var segbots = {};
var numOfFailedSegbots = 0;
var identity = null;
var leader = false;
var segbot = null;
var servo1Cmd = null;
var servo2Cmd = null;
var servo1Pos = 0.0; // {-2,2} but locked to {-1,1}
var servo2Pos = 0.0;
var pingHandler = null;
var pingInterval = 5000; // ms
var moveBaseAction = null;
var topicsClient = null;
var goToLocationClient = null;
var goBesideLocationClient = null;
var rotateClient = null;
var pauseClient = null;
var requestTourClient = null;
var getTourStateClient = null;
var pingTourClient = null;
var leaveTourClient = null;
var speakClient = null;
var deliverClient = null;
var tourState = { tourAllowed: false, tourInProgress: false, tourDuration: 0,
  tourStartTime: 0, lastPingTime: 0};
var tourStateFresh = false;
var topics = null;
var servosEnabled = false;
var robot_v3 = false;
var curr_color = 0;
var audioSourceBuffer = null;
var audioBufferQueue = [];
var bufferLength = 0;
var mediaSource = null;
var audio = null;


// Scavenger Hunt Statuses
var FINISHED = "<span class=\"glyphicon glyphicon-ok\"></span> Done";
var ONGOING  = "<span class=\"glyphicon glyphicon-time\"></span> Ongoing ";
var TODO     = "<span class=\"glyphicon glyphicon-calendar\"></span> To do";

// Map conversions
var map_res = 0.05;
var map_origin_x = -60.45;
var map_origin_y = -23.4;
var map_width = 1857;
var map_height = 573;
var map_marker_offset = 15; //half of image width

var locations = [{
  name: "Shiqi's Office",
  value: "l3_420"
},{
  name: "Jivko's Office",
  value: "l3_432"
},{
  name: "Peter's Office",
  value: "l3_508"
},{
  name: "Lab",
  value: "l3_414b"
},{
  name: "Conference room",
  value: "l3_516"
}];

var doors = [{
  name: "d1",
  value: "d3_432"
},{
  name: "d2",
  value: "d3_414b2"
},{
  name: "d3",
  value: "d3_420"
},{
  name: "d4",
  value: "d3_414a2"
},{
  name: "d5",
  value: "d3_508",
},{
  name: "d6",
  value: "d3_512",
},{
  name: "d7",
  value: "d3_414b1"
}];



// objects
var Segbot = {
  name : "noname",
  ipaddr : "0.0.0.0",
  rosbridgeport : 9090,
  mjpegserverport : 0,
  ros : null,

  connect : function() {
    log("Trying to connect to " + this.ipaddr + " : " + this.rosbridgeport);
    this.ros = new ROSLIB.Ros({
      url : 'ws://' + this.ipaddr + ':' + this.rosbridgeport
    });
    this.ros.on('connection', function() { log('Connection made!'); });
    this.ros.on('error', function(err) {
      log("Connection failed!");
      log(err);
      error("Connection to segbot failed!");
    });
  }
}

// Functions
function error(errorMessage, errorTitle) {
  errorTitle = errorTitle || "Oops! This is embarassing";
  $("#errorTitle").text(errorTitle);
  $("#errorBody").text(errorMessage);
  $(".error-modal").modal();
  log("error: "+errorMessage);
}

function hasTextEncoder() {
  return !!(window.TextEncoder); 
}

function hasMediaSource() {
  return !!(window.MediaSource || window.WebKitMediaSource);
}

function createIdentity() {
  identity = getUUID();
}

function createSegbots() {
  //segbots["localhost"] = createSegbot("localhost", "127.0.0.1", ROSBRIDGEPORT, MJPEGSERVERPORT);
  //segbots["hypnotoad"] = createSegbot("hypnotoad", "hypnotoad.csres.utexas.edu", ROSBRIDGEPORT, MJPEGSERVERPORT);

  var server = "http://nixons-head.csres.utexas.edu:7979/hostsalivejson";
  if (server == "") {
    error("Will not be able to dynamically load robot's IP addresses","Error: No DNS server set");
    return;
  }
  log("Pinging dns server");
  $.getJSON(server, function(data) {
    $.each(data, function(key, val) {
        createIfHasVirtour(key,val,ROSBRIDGEPORT,Object.keys(data).length)
    });
    if (Object.keys(data).length == 0) {
      $(".available_robots").html("<h3>No robots available at this time</h3>");
    }
    
  }).error(function(err) { error("Failed to ping DNS server"); });
}


function createIfHasVirtour(name, ipaddr, rosbridgeport, totalSegbotsNum) {
  log("Checking Rosbridge existance for " + name + " on url " +  ipaddr + " : " + rosbridgeport);
  var testRos  = new ROSLIB.Ros({
    url : 'ws://' + ipaddr + ':' + rosbridgeport
  });
  
  testRos.on('connection', function() { 
    log('Virtour is running on host.\''+name + '\'');
    segbots[name] = createSegbot(name, ipaddr, ROSBRIDGEPORT, MJPEGSERVERPORT);
    available = true;
    testRos.close();
  });
  
  testRos.on('error', function(err) {
    log('Virtour is not running on host.\''+name + '\'');
    numOfFailedSegbots++;
    if (numOfFailedSegbots == totalSegbotsNum) {
      $(".available_robots").html("<h3>No robots available at this time</h3>");
    }
  });
}


function createSegbot(name, ipaddr, rosbridgeport, mjpegserverport) {
  var bot = Object.create(Segbot);
  bot.name = name;
  bot.ipaddr = ipaddr;
  bot.rosbridgeport = rosbridgeport;
  bot.mjpegserverport = mjpegserverport;
  log("Created segbot: " + name + "(" + ipaddr + ":" + rosbridgeport + ")");

  var colors = ["orange", "green", "yellow", "red"];
  var color = colors[curr_color];
  curr_color = (curr_color + 1) % colors.length;

  var rf = '<div class="col-md-3">';
  var rm = '<div class="robot module ' + color + '" robot="' + name + '">';
  //var im = '<img class="img-circle" src="./image/' + name + '.jpg"/>';
  if (imageExists('./image/' + name + '.jpg')) {
    var im = '<img class="img-circle" src="./image/' + name + '.jpg"/>';
  } else {
    var im = '<img class="img-circle" src="./image/irobot.jpg"/>';
  }

  var nm = '<h2>' + name + '</h2>';
  var ed = '</div>';

  var robotdiv = rf + rm + im + nm + ed + ed;

  $(".robot_links").append(robotdiv);

  return bot;
}

function imageExists(imageUrl){
  var http = new XMLHttpRequest();
  http.open('HEAD', imageUrl, false);
  http.send();
  return http.status != 404;
}

function SegBotConnection(ipaddr, rosbridgeport, mjpegserverport) {
  this.ipaddr = ipaddr;
  this.rosbridgeport = rosbridgeport;
  this.mjpegserverport = mjpegserverport;
  this.ros = new ROSLIB.Ros({
    url : 'ws://' + ipaddr + ':' + rosbridgeport
  });
  log("Created new SegBotConnection with " + ipaddr +  ":" + rosbridgeport);
}

function populateLocations() {
  $(locations).each(function(i, val) {
    $('#locationSelect').append($('<option>', {
      value: val.value,
      text: val.name
    }));
    $('#message_room').append($('<option>', {
      value: val.value,
      text: val.name
    }));
  });
  $(doors).each(function(i, val) {
    $('#doorSelect').append($('<option>', {
      value: val.value,
      text: val.name
    }));
  });
}


function subscribeListener(ros) {
  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/listener',
    messageType : 'std_msgs/String'
  });
  log("Added ping listener");

  listener.subscribe(function(message) {
    log('Received message on ' + listener.name + ': ' + message.data);
    listener.unsubscribe();
  });
}

function subscribePoseListener(ros) {
  // setup a listener for the robot pose
  var poseListener = new ROSLIB.Topic({
    ros : ros,
    name : '/amcl_pose',
    messageType : 'geometry_msgs/PoseWithCovarianceStamped',
  });
  log("Added pose listener!");
  poseListener.subscribe(function(pose) {
    x = pose.pose.pose.position.x;
    y = pose.pose.pose.position.y;
    updatePosition(x,y);
  });
}

function subscribeScavengerHuntListener(ros) {
  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/scav_hunt_status',
    messageType : 'bwi_msgs/ScavStatus'
  });
  log("Added scavenger hunt listener");

  listener.subscribe(function(msg) {
    updateScavengerHuntStatus(msg);
  });
}

// Oleg: Need to figure out this thing with the topic. Is it /audio or /audio/audio? why can it change?
function subscribeAudioListner(ros) {
  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/audio/audio',
    messageType : 'audio_common_msgs/AudioData'
  });
  log("Added ping Audio listener");

  listener.subscribe(function(audioChunck) {
    //log('Received message on ' + listener.name + ': ' + audioChunck.data);
    streamAudio(audioChunck);
  });
}

function viewScavengerHunt() {
  $(".scavengerhunt-modal").modal();
}


function updateScavengerHuntStatus(msg) {
  $(".scavengerhunt-table tbody").html("");
  $(".certificates-table tbody").html("");
  
  for (var i = 0; i < msg.names.length; i++) {
    name = msg.names[i];
    switch (msg.statuses[i]) {
      case 1:
        stat = ONGOING;break;
      case 2:
        stat = FINISHED;break;
      case 3:
        stat = TODO;break;
    }
    a_html  = '<tr>';
    a_html += '<td>' + name + '</td>';
    a_html += '<td>' + stat + '</td>';
    if (stat == FINISHED) {
      path = "http://" + segbot.ipaddr + ":" + IMAGESERVERPORT + "/"
      file = msg.certificates[i];
      path += file;
      a_html += '<td>';
      a_html += '<a href="' + path + '" data-lightbox="cert' + i + '">';
      a_html += '<img class="img-thumbnail cert-img" src="' + path + '" alt="' + name + '"/></a>';
      a_html += '</td>';
    }
    a_html += '</tr>';
    //$(".scavengerhunt-table > tbody:last").append('<tr><td>' + name + '</td><td>' + stat + '</td></tr>');
    $(".scavengerhunt-table > tbody:last").append(a_html);
  }
}

function streamAudio(audioChunckB64) {
  var audioChunckAsStr = atob(audioChunckB64.data);
  var chunckUint8Array = new Uint8Array(audioChunckAsStr.length);
  for (var i = 0; i < audioChunckAsStr.length; i++) {
      chunckUint8Array[i] = audioChunckAsStr.charCodeAt(i);
  }
  
  //log("audio started buffer num:" + chunckUint8Array);
  bufferLength += chunckUint8Array.length;
  audioBufferQueue.push(chunckUint8Array);
  if (audioSourceBuffer && !audioSourceBuffer.updating) {
     loadNextBuffer();
  }
}

function onSourceOpen() {
  // this.readyState === 'open'. Add a source buffer that expects webm chunks.
  audioSourceBuffer = mediaSource.addSourceBuffer('audio/mpeg');
  audioSourceBuffer.addEventListener('updateend', loadNextBuffer, false);
  //....
}

function loadNextBuffer (audioChunck) {
  //log("LoadNextBuffer....");
  if ( audioBufferQueue.length > 0) {
    var audioBufferSize = audioBufferQueue.length;
    var mp3Buffer = new Uint8Array(bufferLength);
    bufferLength = 0;
    //log("mp3BufferEmpty" + mp3Buffer);
    buffSize = 0;
    for ( i = 0; i < audioBufferSize; i++) {
      currBuff = audioBufferQueue.shift();
      mp3Buffer.set(currBuff,buffSize);
      buffSize += currBuff.length;
    }
    //log("Loading next buffer....");
    //log("audio buffer" + mp3Buffer);
    audioSourceBuffer.appendBuffer(mp3Buffer);
  
    if (audio.paused) {
      audio.play();
    }
  }
}


     
     


function updatePosition(x, y){
  xp = 100 * ((x - map_origin_x) / map_res - map_marker_offset) / map_width;
  yp = 100 * ((y - map_origin_y) / map_res - map_marker_offset) / map_height;
  yp = 100 - yp;
  yp = yp - 10;
  //yp = yp - 15; // offset for height of image and for css position
  $(".pos-marker").css("left", xp + "%");
  $(".pos-marker").css("top", yp + "%");
}

function publishTopic(ros) {
  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message({
    linear : {
      x : 0.1,
      y : 0.2,
      z : 0.3
    },
    angular : {
      x : -0.1,
      y : -0.2,
      z : -0.3
    }
  });
  cmdVel.publish(twist);
}

function rotateLeft() {
  log("Rotate left");
  if (robot_v3) {
    requestRotate(-0.4); // v3 rotation is defined differently, also is slower
  } else {
    requestRotate(-0.2);
  }
}

function rotateRight() {
  log("Rotate right");
  if (robot_v3) {
    requestRotate(0.4); // v3 rotation is defined differently, also is slower
  } else {
    requestRotate(0.2);
  }
}

function pauseRobot() {
  requestPause(true);
}

function resumeRobot() {
  requestPause(false);
}

function turnLeft() {
  servo1Pos += 0.2;
  servo1Pos = servo1Pos > 1.0 ? 1.0 : servo1Pos;
  servo1Cmd.publish(new ROSLIB.Message({data: servo1Pos}));
  log("Servo1: " + servo1Pos);
}

function turnRight() {
  servo1Pos -= 0.2;
  servo1Pos = servo1Pos < -1.0 ? -1.0 : servo1Pos;
  servo1Cmd.publish(new ROSLIB.Message({data: servo1Pos}));
  log("Servo1: " + servo1Pos);
}

function turnUp() {
  servo2Pos -= 0.2;
  servo2Pos = servo2Pos < 0.4 ? 0.4 : servo2Pos;
  servo2Cmd.publish(new ROSLIB.Message({data: servo2Pos}));
  log("Servo2: " + servo2Pos);
}

function turnDown() {
  servo2Pos += 0.2;
  servo2Pos = servo2Pos > 2.0 ? 2.0 : servo2Pos;
  servo2Cmd.publish(new ROSLIB.Message({data: servo2Pos}));
  log("Servo2: " + servo2Pos);
}
function turnCenter() {
  servo1Pos = 0.0;
  servo2Pos = 0.0;
  servo1Cmd.publish(new ROSLIB.Message({data: 0.0}));
  servo2Cmd.publish(new ROSLIB.Message({data: 1.0}));
  log("Servo1: " + servo1Pos);
  log("Servo2: " + servo2Pos);
}

function showMap() {
  log("Showing map");
  $(".map-modal").modal();
}

function sendGoal(pose) {
  var goal = new ROSLIB.Goal({
    actionClient : moveBaseAction,
    goalMessage : {
      target_pose : {
        header : {
          frame_id : '/base_footprint'
        },
        pose : pose
      }
    }
  });

  goal.on('result', function(result) {
    log("move_base result: " + result);
  });
  goal.send();
}

function requestLocation(locationStr) {
  log('requesting goToLocation: ' + locationStr);
  var request = new ROSLIB.ServiceRequest({ location: locationStr, user: identity});
  goToLocationClient.callService(request, function(result) {
    log('Result for requestLocation service call on '
      + goToLocationClient.name + ': ' + result.result);
    if (result.result == 1) { // success
      alert("Done going to location!");
    } else if (result.result == -1) { // terminated
      log("requestLocation terminated");
    } else if (result.result == -2) { // preempted
      log("requestLocation preempted");
    } else if (result.result == -3) { // aborted
      log("requestLocation aborted");
    }
  });
}

function requestBesideLocation(locationStr) {
  log('requesting goBesideLocation: ' + locationStr);
  var request = new ROSLIB.ServiceRequest({ location: locationStr, user: identity});
  goBesideLocationClient.callService(request, function(result) {
    log('Result for requesBesidetLocation service call on '
      + goToLocationClient.name + ': ' + result.result);
    if (result.result == 1) { // success
      alert("Done going beside location!");
    } else if (result.result == -1) { // terminated
      log("goBesideLocation terminated");
    } else if (result.result == -2) { // preempted
      log("goBesideLocation preempted");
    } else if (result.result == -3) { // aborted
      log("goBesideLocation aborted");
    }
  });
}

function requestRotate(rotateDelta) {
  log('requesting rotate: ' + rotateDelta);
  var request = new ROSLIB.ServiceRequest({ rotateDelta: rotateDelta, user: identity});
  rotateClient.callService(request, function(result) {
    log('Result for requestRotate service call on '
      + rotateClient.name + ': ' + result.result);
  });
}

function requestPause(pause) {
  log('requesting: ' + (pause ? "stop operations" : "resume operations"));
  var type = pause ? 0 : 1; // 0 - pause, 1 - resume
  var request = new ROSLIB.ServiceRequest({ type : type});
  pauseClient.callService(request, function(result) {
    log('Result for requestPause service call on '
      + pauseClient.name + ': ' + result.result);
  });
}

function getUUID() {
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
        var r = Math.random()*16|0, v = c == 'x' ? r : (r&0x3|0x8);
            return v.toString(16);
  });
}

function getTopics() {
  log('getting topics');
  var request = new ROSLIB.ServiceRequest();
  topicsClient.callService(request, function(result) {
    topics = result.topics;

    // enable or disable scavenger hunt
    if (!topicAvailable("/scav_hunt_status")) {
      $(".scavengerHunt").hide();
    }
  });
}

function topicAvailable(topic) {
  return $.inArray(topic, topics) > -1;
}

function getTourState() {
  var request = new ROSLIB.ServiceRequest();
  getTourStateClient.callService(request, function(result) {

    tourState = { tourAllowed: result.tourAllowed, tourInProgress: result.tourInProgress,
      tourDuration: result.tourDuration, tourStartTime: result.tourStartTime,
      lastPingTime: result.lastPingTime};

    tourStateFresh = true;

    if (tourState.tourAllowed && !tourState.tourInProgress) {
      $(".leaderText").text("Tour available!");
      $(".requestTour").show();
      $(".leaveTour").hide();
      $(".leaderControl").height(60);
      $(".leaderControl").css("top", "710px");
    } else if (leader) {
      $(".leaderText").text("You are controlling the tour!");
      $(".requestTour").hide();
      $(".leaveTour").show();
      $(".leaderControl").height(60);
      $(".leaderControl").css("top", "710px");
    } else {
      if (!tourState.tourAllowed) {
        $(".leaderText").text("Sorry, tour leaders are not enabled at this time.");
      } else {
        $(".leaderText").text("Someone is controlling the tour, enjoy!");
      }
      $(".requestTour").hide();
      $(".leaveTour").hide();
      $(".leaderControl").height(20);
      $(".leaderControl").css("top", "750px");
    }
  });
}

function requestTour() {
  log('requesting tour');
  var request = new ROSLIB.ServiceRequest({ user: identity });
  requestTourClient.callService(request, function(result) {
    if (result.result > 0) { //success
      leader = true;
      showControls();
      getTourState();
      pingHandler = window.setInterval(pingTour, pingInterval);
    } else if (result.result == ERROR_NOTOURALLOWED) {
      alert("Sorry! No tour allowed");
    } else if (result.result == ERROR_TOURINPROGRESS) {
      alert("Sorry! There is already a tour in progress");
    } else {
      alert("Sorry! There was an error ("+result.result+")");
    }
  });
}

function sayMessage(message) {
  var request = new ROSLIB.ServiceRequest({ message: message });
  speakClient.callService(request, function(result) {
    log("Spoke message");
  });
}

function deliverMessage(message, loc) {
  var request = new ROSLIB.ServiceRequest({ message: message, location: loc });
  deliverClient.callService(request, function(result) {
    log("Delivered message");
  });
}

function pingTour() {
  var request = new ROSLIB.ServiceRequest({ user: identity });
  pingTourClient.callService(request, function(result) {
    if (result.result > 0) {
    } else if (result.result == ERROR_NOTOURALLOWED) {
      alert("ping failed: no tour allowed");
    } else if (result.result == ERROR_TOURINPROGRESS) {
      alert("ping failed: tour in progress");
    } else if (result.result == ERROR_NOTTOURLEADER) {
      alert("ping failed: not tour leader");
    } else if (result.result == ERROR_NOTOURINPROGRESS) {
      alert("ping failed: no tour in progress");
    }
  });
}

function leaveTour() {
  log('leaving tour');
  var request = new ROSLIB.ServiceRequest({ user: identity });
  leaveTourClient.callService(request, function(result) {
    if (result.result > 0) {
      leader = false;
      hideControls();
      window.clearInterval(pingHandler);
      getTourState();
      log('left tour successfully');
    }
  });
  return true;
}

function showControls() {
  if (servosEnabled) {
    $(".servoControl").fadeIn();
  }
  $(".rotateControl").fadeIn();
  $(".pauseControl").fadeIn();
  $(".locationForm").fadeIn();
  $(".navigateBtn").fadeIn();
  $(".messageControl").fadeIn();
}

function hideControls() {
  $(".servoControl").hide();
  $(".rotateControl").hide();
  $(".pauseControl").hide();
  $(".locationForm").hide();
  $(".navigateBtn").hide();
  $(".messageControl").hide();
}

// Page Handlers
$(document).ready(function() {
  log("Loaded.");

  log("Creating identity");
  createIdentity();

  log("Creating segbots");
  createSegbots();

  log("Populating locations");
  populateLocations();

  $(".requestTour").hide();
  $(".leaveTour").hide();

  setBeforeUnload();
});

function setBeforeUnload() {
  window.onbeforeunload = function() {
    if (leader) {
      leaveTour();
      return
    } else {
      return;
    }
  };
}

function clearBeforeUnload() {
  window.onbeforeunload = null;
}

$(".robots").on("click", ".robot", function() {
  var botname = $(this).attr("robot");
  segbot = segbots[botname];

  log("Selected: " + botname); 
  segbot.connect();

  log("Subscribing listeners");
  subscribeListener(segbot.ros);
  subscribePoseListener(segbot.ros);
  subscribeScavengerHuntListener(segbot.ros);
  subscribeAudioListner(segbot.ros);


  // hide the intro stuff
  $(".intro").fadeOut();
  $(".robots").fadeOut();

  // hide the controls
  hideControls();

  // set up title
  $(".controllingText").text("Viewing " + botname);

  // set up service client for getting list of topics 
  topicsClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/rosapi/topics',
    serviceType : 'rosapi/Topics'
  });

  // get topics
  getTopics();

  // set up topic for controlling servo
  servo1Cmd = new ROSLIB.Topic({
    ros : segbot.ros,
    name : '/servo0_cmd',
    messageType : 'std_msgs/Float32'
  });
  servo2Cmd = new ROSLIB.Topic({
    ros : segbot.ros,
    name : '/servo1_cmd',
    messageType : 'std_msgs/Float32'
  });

  // set up actionlib for teleop moving
  moveBaseAction = ROSLIB.ActionClient({
    ros : segbot.ros,
    serverName : '/move_base',
    actionName : 'move_base_msgs/MoveBaseAction'
  });

  // set up service client for sending location commands
  goToLocationClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/go_to_location',
    serviceType : 'bwi_virtour/GoToLocation'
  });
  goBesideLocationClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/go_beside_location',
    serviceType : 'bwi_virtour/GoBesideLocation'
  });

  // set up service client for sending location commands
  rotateClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/rotate',
    serviceType : 'bwi_virtour/Rotate'
  });

  // set up service client for requesting pausing
  pauseClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/scav_control',
    serviceType: 'bwi_msgs/ScavHunt'
  });

  // set up service client for requesting tours
  requestTourClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/tour_manager/request_tour',
    serviceType : 'bwi_virtour/RequestTour'
  });

  // set up service client for getting tour state
  getTourStateClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/tour_manager/get_tour_state',
    serviceType : 'bwi_virtour/GetTourState'
  });

  // set up service client for pinging the tour
  pingTourClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/tour_manager/ping_tour',
    serviceType : 'bwi_virtour/PingTour'
  });

  // set up service client for leaving the tour
  leaveTourClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/tour_manager/leave_tour',
    serviceType : 'bwi_virtour/LeaveTour'
  });

  // set up service client for speaking a message
  speakClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/speak_message_service/speak_message',
    serviceType : 'bwi_services/SpeakMessage'
  });

  // set up service client for delivering a message
  deliverClient = new ROSLIB.Service({
    ros : segbot.ros,
    name : '/deliver_message',
    serviceType : 'bwi_services/DeliverMessage'
  });

  // reset the servo
  turnCenter();

  // show the robot stuff
  $(".control").delay(800).fadeIn();

  // tour setup
  getTourState();

  // enable or disable servos
  servosEnabled = topicAvailable("/servo0_status") && topicAvailable("/servo1_status");

  // check if it's a v2 or v3
  robot_v3 = topicAvailable("/velodyne_points");

  // set up video streaming
  var videoTopic = "";
  if (topicAvailable("/camera/image_raw")) {
    videoTopic = "/camera/image_raw";
    //videoTopic += "?invert";
    log("Using /camera/image_raw for video source");
  } else {
    videoTopic = "/nav_kinect/rgb/image_raw";
    log("Using /nav_kinect/rgb/image_raw for video source");
  }
  var videoSource = "http://" + segbot.ipaddr + ":" + segbot.mjpegserverport
                      + "/stream?topic=" + videoTopic + "&quality=" + VIDEO_QUALITY;
  log("Loading video from: " + videoSource);
  $(".controllingIframe").append("<img width=\"100%\" height=\"100%\" src=\"" + videoSource + "\">");

  // set up video streaming (mp3 format)
  if (hasMediaSource() && hasTextEncoder()) {
    log("has media source, start streaming audio.");
     //$(".controllingIframe").append("<audio id=\"audio\" controls=\"controls\">");
     //$(".controllingIframe").append("<source id=\"mp3Source\" type=\"audio/mp3\"></source>");
     //$(".controllingIframe").append("</audio>");
     //audio = document.querySelector('audio');
     $(".controllingIframe").append("<audio id=\"mp3Source\" controls=\"controls\"></audio>");
     audio = document.getElementById('mp3Source');
     
     window.MediaSource = window.MediaSource || window.WebKitMediaSource;
     mediaSource = new MediaSource();

     audio.src= window.URL.createObjectURL(mediaSource);
     
     mediaSource.addEventListener('sourceopen', onSourceOpen, false);
     mediaSource.addEventListener('webkitsourceopen', onSourceOpen, false);
     
  } else {
    alert("Bummer. Your browser doesn't support the MediaSource API or the TextEncoder feature! Cannot stream audio.");
  }

});

// add callback handlers for buttons
$(".turnLeft").click(function() {turnLeft();});
$(".turnRight").click(function() {turnRight();});
$(".turnUp").click(function() {turnUp();});
$(".turnDown").click(function() {turnDown();});
$(".turnCenter").click(function() {turnCenter();});
$(".labimage").click(function() {showMap();});
$(".rotateRight").click(function() {rotateRight();});
$(".rotateLeft").click(function() {rotateLeft();});
$(".pauseBtn").click(function() {pauseRobot();});
$(".resumeBtn").click(function() {resumeRobot();});

$(".getTourStatus").click(function() {
  getTourState();
});

$(".requestTour").click(function() {
  getTourState();
  requestTour();
  getTourState();
});

$(".leaveTour").click(function() {
  getTourState();
  leaveTour();
  getTourState();
});

$(".speakBtn").click(function() {
  $(".message_destination").hide();
  $(".message_say").show();
  $(".message_deliver").hide();
  $(".message-modal").modal();
});

$(".deliverBtn").click(function() {
  $(".message_destination").show();
  $(".message_say").hide();
  $(".message_deliver").show();
  $(".message-modal").modal();
});

$(".message_say").click(function() {
  msg = $("#message_body").val();
  if (msg == "") {
    alert("Message is required");
    return;
  }
  log("Speaking message: "+msg);
  sayMessage(msg);
  $("#message_body").val("");
  $("#message_room").val("");
  $(".message-modal").modal("hide");
});

$(".message_deliver").click(function() {
  msg = $("#message_body").val();
  loc = $("#message_room").val();
  if (msg == "" && loc == "") {
    alert("Message and location are required");
    return;
  } else if (msg == "") {
    alert("Message is required");
    return;
  } else if (loc == "") {
    alert("Location is required");
    return;
  }
  log("Delivering message: " + msg + " to room: " + loc);
  deliverMessage(msg, loc);
  $("#message_body").val("");
  $("#message_room").val("");
  $(".message-modal").modal("hide");
});

$(".viewScavengerHunt").click(function() {
  viewScavengerHunt();
});

$("#locationSelect").change(function() {
  $("#doorSelect").val("");
});

$("#doorSelect").change(function() {
  $("#locationSelect").val("");
});

// add callback handlers for navigate form
$(".navigateBtn").click(function() {
  var place = $("#locationSelect").val();
  var door = $("#doorSelect").val();

  if (place == "" && door == "") { // if nothing is selected
    alert("Please select a location");
    return;
  } else if (place == "") { // if a door is selected
    requestBesideLocation(door);
  } else { // a place is selected
    requestLocation(place); 
  }

  $(".map-modal").modal("hide");
});

$(".reloadBtn").click(function() {
  log("reloading page");
  location.reload();
});

// map arrow keys to movement
$(document).keypress(function(e) {
  if (e.keyCode === 37) {
    turnLeft();
  } else if (e.keyCode === 39) {
    turnRight();
  } else if (e.keyCode === 38) {
    turnUp();
  } else if (e.keyCode === 40) {
    turnDown();
  }
});
