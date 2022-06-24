import React from "react";
import "./Xbox.css";
import Gamepad from "react-gamepad";
// import Draggable from "react-draggable";

const AXIS_THROTTLE = 10;

const CONTROLLER_FUCTIONS = {
  'XLinear': (state, commands) => {
    commands.movement.angular[2] = -0.3 * deadzone(state.LeftStickX)
    return commands},
  'ZLinear': (state, commands) => {
    commands.movement.linear[2] = -3 * state.LeftTrigger;
    if (state.RightTrigger != 0) commands.movement.linear[2] = 2 * state.RightTrigger;
    return commands},
  'YLinear': (state, commands) => {
    commands.movement.linear[1] = -2 * deadzone(state.LeftStickY)
    return commands},
  'XAngular': (state, commands) => {
    commands.movement.angular[0] = -0.3 * deadzone(state.RightStickY)
    return commands},
  'YAngular': (state, commands) => {
    if (state.LB) commands.movement.angular[1]= 0.1;
    else if (state.RB) commands.movement.angular[1] = -0.1;
    else commands.movement.angular[1] = 0;
    return commands},
  'ZAngular': (state, commands) => {
    commands.movement.angular[1] = 0.3 * deadzone(state.RightStickX)
    return commands},
  'Manipulator': (state, commands, socket) => {
    if (state.A) socket.emit("Set Manipulator Angle", 100);
    if (state.B) socket.emit("Set Manipulator Angle", 0);
    //if (state.LB) socket.emit("Set Manipulator Velocity", 1);
    //else if (state.RB) socket.emit("Set Manipulator Velocity", -1);
    //else socket.emit("Stop Manipulator");
    //console.log("pain" + state.A)
    return commands}
}

const deadzone = (value, tol=0.2) => {
  if(Math.abs(value) < tol) return 0;
  return value;
}

export default class Xbox extends React.Component {
  state = {
    A: false,
    B: false,
    X: false,
    Y: false,
    Start: false,
    Back: false,
    LT: false,
    RT: false,
    LB: false,
    RB: false,
    LS: false,
    RS: false,
    DPadUp: false,
    DPadDown: false,
    DPadLeft: false,
    DPadRight: false,
    RightTrigger: 0,
    LeftTrigger: 0,
    LeftStickX: 0,
    LeftStickY: 0,
    RightStickX: 0,
    RightStickY: 0,
  };

  vect = {
    linear: [0, 0, 0],
    angular: [0, 0, 0]
  }

  camera_index = 0;

  BUTTON_OPACITY = {
    true: "50%",
    false: "100%",
  };

  BUTTON_TOP = {
    true: 7,
    false: 0,
  };

  topOffset = {
    A: 243,
    B: 187 + this.BUTTON_TOP[this.state.B],
    X: 187 + this.BUTTON_TOP[this.state.X],
    Y: 135 + this.BUTTON_TOP[this.state.Y],
    LeftStickY: 175,
    RightStickY: 296
  }

  leftOffset = {
    LeftStickX: 98,
    RightStickX: 394
  }

  constructor(props) {
    super();
    this.handleChange = this.handleChange.bind(this);
    this.handleAxis = this.handleAxis.bind(this);
  }

  handleChange(buttonName, pressed) {
    let change = {};
    change[buttonName] = pressed;
    this.setState(change);

    if(pressed && buttonName == "Start") {
      this.props.socket.emit("Arm Motors", "please");
    }
  }

  getTriggerStyle(value) {
    return {
      filter: "invert(" + Math.abs(value) + ")",
    }
  }

  getTopOffsetStyle(value) {
    return {top: value + "px"}
  }

  getLeftOffsetStyle(value) {
    return {left: value + "px"}
  }

  getOpacity(button) {
    return {opacity: this.BUTTON_OPACITY[button]}
  }

  handleAxis(axisName, value, previousValue) {
    let rounded = Math.round(value * AXIS_THROTTLE) / AXIS_THROTTLE
    if(Math.abs(this.state[axisName] - rounded) > 0) {
      let change = {};
      change[axisName] = rounded;
      this.setState(change);
    }
  }

  handleControllerFunctions(functionHash) {
    let tempCommands = {
      movement: this.vect,
    }
    for (let key in functionHash) {
      tempCommands = functionHash[key](this.state, tempCommands, this.props.socket);
    }
    this.vect = tempCommands.movement;

    this.sendCommands()
  }

  updateCameraIndex() {
      let currIndex = this.camera_index;
      if(this.state.DPadUp) currIndex = 0;
      else if(this.state.DPadRight) currIndex = 1;
      else if(this.state.DPadDown) currIndex = 2;
      else if(this.state.DPadLeft) currIndex = 3;

      if(currIndex != this.camera_index) {
        this.camera_index = currIndex;
        this.props.socket.emit("Set Camera", this.camera_index);
      }
  }

  componentDidUpdate() {
    //this.updateVects();
    this.handleControllerFunctions(CONTROLLER_FUCTIONS);
    this.updateCameraIndex();
  }


  sendCommands() {
    this.props.socket.emit("Send Movement", this.vect);
  }

  render() {
    return (
      <Gamepad
        onButtonChange={this.handleChange}
        onAxisChange={this.handleAxis}
      >
        <div className="gamepad">
          <img src="/xboxImages/bg.png" id="bg" />
          <img src="/xboxImages/a.png" id="a" style={
            {
              ...this.getOpacity(this.state.A),
              ...this.getTopOffsetStyle(this.topOffset.A + this.BUTTON_TOP[this.state.A])
            }} />
          <img src="/xboxImages/b.png" id="b" style={
            {
              ...this.getOpacity(this.state.B),
              ...this.getTopOffsetStyle(this.topOffset.B + this.BUTTON_TOP[this.state.B])
            }} />
          <img src="/xboxImages/x.png" id="x" style={
            {
              ...this.getOpacity(this.state.X),
              ...this.getTopOffsetStyle(this.topOffset.X + this.BUTTON_TOP[this.state.X])
            }} />
          <img src="/xboxImages/y.png" id="y" style={
            {
              ...this.getOpacity(this.state.Y),
              ...this.getTopOffsetStyle(this.topOffset.Y + this.BUTTON_TOP[this.state.Y])
            }} />
          <img
            src="/xboxImages/left.png"
            id="left"
            style={this.getOpacity(this.state.DPadLeft)}
          />
          <img
            src="/xboxImages/right.png"
            id="right"
            style={this.getOpacity(this.state.DPadRight)}
          />
          <img src="/xboxImages/up.png" id="up" style={this.getOpacity(this.state.DPadUp)} />
          <img
            src="/xboxImages/down.png"
            id="down"
            style={this.getOpacity(this.state.DPadDown)}
          />
          <img
            src="/xboxImages/stick.png"
            id="stick"
            style={{
              ...this.getLeftOffsetStyle(this.leftOffset.LeftStickX + this.state.LeftStickX * 20),
              ...this.getTopOffsetStyle(this.topOffset.LeftStickY + this.state.LeftStickY * -20)
            }}
          />{" "}
          <img
            src="/xboxImages/stick2.png"
            id="stick2"
            style={{
              ...this.getLeftOffsetStyle(this.leftOffset.RightStickX + this.state.RightStickX * 20),
              ...this.getTopOffsetStyle(this.topOffset.RightStickY + this.state.RightStickY * -20)
            }}
          />{" "}
          <img
            src="/xboxImages/bumperleft.png"
            id="bumpl"
            style={this.getOpacity(this.state.LB)}
          />
          <img
            src="/xboxImages/bumperright.png"
            id="bumpr"
            style={this.getOpacity(this.state.RB)}
          />
          <img
            src="/xboxImages/lt.png"
            id="lt"
            style={this.getTriggerStyle(this.state.LeftTrigger)}
          />
          <img
            src="/xboxImages/rt.png"
            id="rt"
            style={this.getTriggerStyle(this.state.RightTrigger)}
          />
          <img src="/xboxImages/left.png" id="back" style={this.getOpacity(this.state.Back)} />
          <img
            src="/xboxImages/right.png"
            id="start"
            style={this.getOpacity(this.state.Start)}
          />
        </div>
      </Gamepad>
    );
  }
}
