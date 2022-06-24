import React, {Component} from 'react';

class Float extends React.Component {
  state = {
    speed: "",
    direction: "",
    time: ""
  };

  constructor(props) {
    super(props);
  }

  resurface = () => {
    let dir = this.state.direction
    if (dir < 0) {
      dir = 360 + this.state.direction;
    }

    dir = 90 - this.state.direction;
    if (dir < 0) {
      dir += 360;
    }

    let distance = this.state.speed * 3.6 * this.state.time;

    let realDir = dir * Math.PI / 180;
    let xChange = distance * Math.cos(realDir);
    let yChange = distance * Math.sin(realDir);

    if (realDir === 0 || realDir === Math.PI) {
      yChange = 0;
      xChange = distance;
    }
    if (realDir === Math.PI/2 || realDir === 3*Math.PI/2) {
      xChange = 0;
      yChange = distance;
    }

    return {
      distance: distance,
      xChange: xChange,
      yChange: yChange
    };
  }

  render() {
    let data = this.resurface();
    return(
      <div>
        <textarea
          onChange={(e) => {this.setState({direction: e.target.value});}}
          value={this.state.direction}
          placeholder="direction"
        />
        <textarea
          onChange={(e) => {this.setState({speed: e.target.value});}}
          value={this.state.speed}
          placeholder="speed"
        />
        <textarea
          onChange={(e) => {this.setState({time: e.target.value});}}
          value={this.state.time}
          placeholder="time"
        />
      {/* <button onClick={() => {
        this.setState({
          speed: parseFloat(this.state.speed),
          direction: parseFloat(this.state.direction),
          time: parseFloat(this.state.time)
        });
      }}>Submit</button> */}
      <p>Total Distance: {data.distance} km</p>
      <p>X Distance: {data.xChange} km, horizontal boxes: {Math.trunc(data.xChange / 2)}</p>
      <p>Y Distance: {data.yChange} km, vertical boxes: {Math.trunc(data.yChange / 2)}</p>
    </div>
    );

  }
}

export default Float;