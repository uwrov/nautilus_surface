import React from 'react';

/**
 * UserWebcam component. Access webcam, display it as a widget, and publish feed back to ROS
 * @author: Kenneth Yang
 * @version: 1.0.0
 */
export default class UserWebcam extends React.Component {
    /**
     * Create video src state and connect to SocketIO
     * @constructor
     */
    constructor(props) {
        super(props);
        this.socket = require('socket.io-client')('http://localhost:4040');
        this.state = {
            frame: null,
        }
    }

    /**
     * Get webcam stream and set it to video src. Set video to 480p.
     */
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

        // Get video stream via getUserMedia
        navigator.mediaDevices.getUserMedia(videoConstraints).then(stream => {
            let video = document.querySelector('video');
            video.srcObject = stream;
        }).catch(err => {
            console.log(err);
        });
    }

    /**
     * Display webcam as video element
     */
    render() {
        return (
            <div className="user-webcam-video">
                <video id="video" autoPlay={true}/>
            </div>
        );
    }
}