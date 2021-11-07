import React from 'react';

export default class UserWebcam extends React.Component {
    constructor(props) {
        super(props);
        this.socket = require('socket.io-client')('http://localhost:4040');
        this.state = {
            videoSrc: null,
        }
    }

    componentDidMount() {
        let videoConstraints = {
            video: {
                width: {
                    ideal: 640
                },
                height: {
                    ideal: 480
                }
            }
        };
        navigator.mediaDevices.getUserMedia(videoConstraints).then(stream => {
            let video = document.querySelector('video');
            video.srcObject = stream;
        }).catch(err => {
            console.log(err);
        });
    }

    render() {
        return (
            <div className="user-webcam-video">
                <video id="video" autoPlay={true}/>
            </div>
        );
    }
}