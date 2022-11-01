// Copy paste modified from Pololu's 32U4 A-Star thing
// https://www.pololu.com/blog/577/building-a-raspberry-pi-robot-with-the-a-star-32u4-robot-controller

// Copyright Pololu Corporation.  For more information, see https://www.pololu.com/

stop_motors = true;
block_set_motors = false;
mouse_dragging = false;

function init() {
  poll();
  $("#joystick").bind("touchstart",touchmove);
  $("#joystick").bind("touchmove",touchmove);
  $("#joystick").bind("touchend",touchend);
  $("#joystick").bind("mousedown",mousedown);
  $(document).bind("mousemove",mousemove);
  $(document).bind("mouseup",mouseup);
}


function poll() {
  if(stop_motors && !block_set_motors)
  {
    setMotors(0,0);
    stop_motors = false;
  }
}

function touchmove(e) {
  e.preventDefault();
  var touch = e.originalEvent.touches[0] || e.originalEvent.changedTouches[0];
  dragTo(touch.pageX, touch.pageY);
}

function mousedown(e) {
  e.preventDefault();
  mouse_dragging = true;
}

function mouseup(e) {
  if(mouse_dragging)
  {
    e.preventDefault();
    mouse_dragging = false;
    stop_motors = true;
  }
}

function mousemove(e) {
  if(mouse_dragging)
  {
    e.preventDefault();
    dragTo(e.pageX, e.pageY);
  }
}

function dragTo(x, y) {
  var elm = $('#joystick').offset();
  var w = $('#joystick').width();
  var h = $('#joystick').height();
  x = x - elm.left;
  y = y - elm.top;
  

  x = (x-w/2.0)/(w/2.0);
  y = (y-h/2.0)/(h/2.0);

  if(x < -1) x = -1;
  if(x > 1) x = 1;
  if(y < -1) y = -1;
  if(y > 1) y = 1;

  var fwd = Math.round(-100*y);
  var rot = Math.round(100*x);

  if(fwd > 100) fwd = 100;
  if(fwd < -100) fwd = -100;

  if(rot > 100) rot = 100;
  if(rot < -100) rot = -100;

  stop_motors = false;
  setMotors(fwd, rot);
}

function touchend(e) {
  e.preventDefault();
  stop_motors = true;
}

function setMotors(fwd, rot) {
  $("#display").html("Forward: " + fwd + "%, Rotate: "+ rot + "%");

  if(block_set_motors) return;
  block_set_motors = true;

  $.ajax({url: "motors/"+fwd+","+rot}).done(setMotorsDone);
}

function setMotorsDone() {
  block_set_motors = false;
}