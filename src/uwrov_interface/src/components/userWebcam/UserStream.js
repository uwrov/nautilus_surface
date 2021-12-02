import React from "react";

export default class UserStream extends React.Component {
  constructor(props) {
    super(props);
    this.socket = require('socket.io-client')('http://localhost:4040');
    this.state = {
      src: "https://hips.hearstapps.com/hmg-prod.s3.amazonaws.com/images/funny-dog-captions-1563456605.jpg",
      timestamp: 1638334180.820340,
    };

    this.socket.on("Text", this.displayVideo);
  }

  displayVideo = (data) => {
    function encode (input) {
        var keyStr = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";
        var output = "";
        var chr1, chr2, chr3, enc1, enc2, enc3, enc4;
        var i = 0;
    
        while (i < input.length) {
            chr1 = input[i++];
            chr2 = i < input.length ? input[i++] : Number.NaN; // Not sure if the index 
            chr3 = i < input.length ? input[i++] : Number.NaN; // checks are needed here
    
            enc1 = chr1 >> 2;
            enc2 = ((chr1 & 3) << 4) | (chr2 >> 4);
            enc3 = ((chr2 & 15) << 2) | (chr3 >> 6);
            enc4 = chr3 & 63;
    
            if (isNaN(chr2)) {
                enc3 = enc4 = 64;
            } else if (isNaN(chr3)) {
                enc4 = 64;
            }
            output += keyStr.charAt(enc1) + keyStr.charAt(enc2) +
                      keyStr.charAt(enc3) + keyStr.charAt(enc4);
        }
        return output;
    }

    setInterval(() => {
        let src = 'data:image/png;base64,'+encode(new Uint8Array(data.user_video));
        let timestamp = data.timestamp;
        let canvas = document.getElementById('image-stream');
        let newImg = document.createElement('img');
        newImg.src = src;
        if (canvas.firstChild && timestamp > this.state.timestamp) {
            canvas.replaceChild(newImg, canvas.firstChild)
            this.setState({ src: src, timestamp: timestamp });
        }
    }, 500);
  }

  render() {
    return (
      <div id="image-stream">
          <img src=''></img>
      </div>
    );
  }
}