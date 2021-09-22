import React from "react";

export default class Text extends React.Component {
  constructor(props) {
    super(props);
    this.socket = require('socket.io-client')('http://localhost:4040')
    this.state = {
      text: "Placeholder text"
    };

    this.socket.on("Text", this.updateText);
  }

  updateText = (data) => {
    console.log("got text update: " + data.text);
    this.setState({ text: data.text });
  }

  render() {
    return (
      <div className="text-box">
        <p className="text">{this.state.text}</p>
      </div>
    );
  }
}